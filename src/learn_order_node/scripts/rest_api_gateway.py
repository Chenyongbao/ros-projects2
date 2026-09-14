#!/usr/bin/env python3
"""学习项目的最小 REST -> ROS 2 Service 网关。

这个进程只负责协议转换，不直接访问 MissionQueue，也不创建 Action Client：

HTTP JSON -> SubmitOrder Service -> order_gateway_node

这样 REST 客户端和 ROS 任务中枢之间只有一个稳定的订单入口。
"""

import argparse
from collections import OrderedDict
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
from typing import Any, Dict
from urllib.parse import parse_qs, urlparse

import rclpy
from rclpy.node import Node

from learn_order_interfaces.srv import CancelMission
from learn_order_interfaces.srv import GetMissionEvents
from learn_order_interfaces.srv import GetMissionStatus
from learn_order_interfaces.srv import SubmitOrder
from std_srvs.srv import Trigger


class SubmitOrderClient(Node):
    """把 HTTP 操作转换为已有的 ROS 2 Service 请求。"""

    def __init__(self, service_name: str, timeout_sec: float) -> None:
        super().__init__("rest_submit_order_client")
        self._submit_client = self.create_client(SubmitOrder, service_name)
        self._dispatch_client = self.create_client(Trigger, "/dispatch_next")
        self._cancel_client = self.create_client(CancelMission, "/cancel_mission")
        self._status_client = self.create_client(GetMissionStatus, "/mission_status")
        self._events_client = self.create_client(GetMissionEvents, "/mission_events")
        self._timeout_sec = timeout_sec
        # rclpy executor 不应被多个 HTTP 线程同时 spin，因此串行保护 Service 调用。
        self._call_lock = threading.Lock()

    def _call(self, client: Any, request: Any) -> Any:
        """同步等待一次 ROS Service 调用；HTTP 线程由外层锁串行化。"""
        if not client.wait_for_service(timeout_sec=self._timeout_sec):
            return None, {
                "success": False,
                "error_code": "ROS_SERVICE_UNAVAILABLE",
                "message": "ROS service is unavailable",
                "status_code": 503,
            }
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._timeout_sec)
        if not future.done():
            return None, {
                "success": False,
                "error_code": "ROS_SERVICE_TIMEOUT",
                "message": "ROS service call timed out",
                "status_code": 504,
            }
        response = future.result()
        if response is None:
            return None, {
                "success": False,
                "error_code": "ROS_EMPTY_RESPONSE",
                "message": "ROS service returned no response",
                "status_code": 502,
            }
        return response, None

    def submit(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """提交订单并返回可直接编码为 HTTP JSON 的字典。"""
        with self._call_lock:
            request = SubmitOrder.Request()
            request.order_id = str(payload.get("order_id", ""))
            request.order_type = str(payload.get("order_type", "transport"))
            request.priority = int(payload.get("priority", 0))
            request.tags = [str(tag) for tag in payload.get("tags", [])]

            # HTTP 层接收结构化 payload；ROS Service 的契约暂时传输 JSON 字符串。
            business_payload = payload.get("payload", payload.get("payload_json", {}))
            if isinstance(business_payload, str):
                request.payload_json = business_payload
            else:
                request.payload_json = json.dumps(
                    business_payload, separators=(",", ":"), sort_keys=True
                )

            response, error = self._call(self._submit_client, request)
            if error is not None:
                return error

            return {
                "success": bool(response.accepted),
                "accepted": bool(response.accepted),
                "error_code": "NONE" if response.accepted else "ORDER_REJECTED",
                "reject_reason": response.reject_reason,
                "mission_id": response.mission_id,
                "queue_size": int(response.queue_size),
                "message": response.message,
                # HTTP 200 表示网关完成了调用；订单被业务拒绝不属于网关故障。
                "status_code": 200,
            }

    def trigger(self) -> Dict[str, Any]:
        """调用 /dispatch_next（无参数 Trigger Service），手动派发队列下一个任务。

        返回的 success 表示“派发动作是否成功执行了一个任务”；
        队列为空或 Action Server 未就绪时 success=false，属正常业务结果。
        """
        with self._call_lock:
            response, error = self._call(self._dispatch_client, Trigger.Request())
            if error is not None:
                return error
            return {
                "success": bool(response.success),
                "error_code": "NONE" if response.success else "OPERATION_REJECTED",
                "message": response.message,
                # HTTP 恒为 200：调用本身完成，业务结果看 success 字段。
                "status_code": 200,
            }

    def cancel(self, mission_id: str) -> Dict[str, Any]:
        """按 mission_id 取消任务，对应 /cancel_mission（CancelMission Service）。

        语义与 C++ 节点一致：
          - mission_id 非空且任务在排队中 -> 同步取消，返回时任务已落 CANCELED 终态；
          - mission_id 非空且任务在执行中 -> 只发起 Goal 取消，
            最终 CANCELED 由 Action 结果异步确认，可再调 status() 跟踪；
          - mission_id 为空 -> 兼容旧行为：取消当前活动任务。
        """
        with self._call_lock:
            request = CancelMission.Request()
            request.mission_id = mission_id
            response, error = self._call(self._cancel_client, request)
            if error is not None:
                return error
            return {
                "success": bool(response.success),
                "error_code": "NONE" if response.success else "CANCEL_REJECTED",
                # 失败原因分类：not_found / already_terminal / no_active_mission / not_in_queue。
                "reject_reason": response.reject_reason,
                "message": response.message,
                "status_code": 200,
            }

    def status(self, mission_id: str) -> Dict[str, Any]:
        """查询单个任务的最新状态与是否终态，对应 /mission_status。

        数据来自 C++ 网关节点的事件历史（取该任务最近一条事件），
        因此任务结束后仍可查询；is_terminal=false 时 timestamp 恒为 0，
        不代表任务未开始，只是尚未进入终态。
        """
        with self._call_lock:
            request = GetMissionStatus.Request()
            request.mission_id = mission_id
            response, error = self._call(self._status_client, request)
            if error is not None:
                return error
            return {
                # success 表示“查询调用成功”，与任务是否存在无关。
                "success": True,
                "found": bool(response.found),
                "error_code": "NONE" if response.found else "MISSION_NOT_FOUND",
                # 最新状态名：QUEUED / DISPATCHED / RUNNING / RETRY_QUEUED /
                # CANCEL_REQUESTED，或终态 SUCCEEDED / FAILED / CANCELED。
                "state": response.state,
                "is_terminal": bool(response.is_terminal),
                "retry_count": int(response.retry_count),
                # 终态时刻（来自终态事件的时间戳）；非终态为 0。
                "timestamp_sec": int(response.timestamp.sec),
                "timestamp_nanosec": int(response.timestamp.nanosec),
                "message": response.message,
                "status_code": 200,
            }

    def events(self, mission_id: str) -> Dict[str, Any]:
        """查询事件历史；筛选逻辑仍由 C++ 节点和核心库负责。"""
        request = GetMissionEvents.Request()
        request.mission_id = mission_id
        with self._call_lock:
            response, error = self._call(self._events_client, request)
            if error is not None:
                return error
            return {
                "success": bool(response.success),
                "error_code": "NONE" if response.success else "EVENT_QUERY_FAILED",
                "message": response.message,
                "events": list(response.events),
                "status_code": 200,
            }


class RestHandler(BaseHTTPRequestHandler):
    """提供健康检查、订单提交、控制和事件查询四类 HTTP 入口。"""

    server_version = "learn-order-rest/0.1"

    def log_message(self, format: str, *args: Any) -> None:
        if self.server.verbose:
            super().log_message(format, *args)

    def send_json(self, status_code: int, payload: Dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, Idempotency-Key")
        self.end_headers()
        self.wfile.write(body)

    def _idempotency_key(self) -> str:
        """读取客户端提供的幂等键；没有键时保持普通请求语义。"""
        return self.headers.get("Idempotency-Key", "").strip()

    @staticmethod
    def _fingerprint(payload: Dict[str, Any]) -> str:
        """把请求内容规范化，检查同一个键是否被复用到另一份订单。"""
        return json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))

    def _submit_with_idempotency(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """执行订单提交，并对 Idempotency-Key 重放原响应。"""
        key = self._idempotency_key()
        if not key:
            return self.server.submit_client.submit(payload)

        fingerprint = self._fingerprint(payload)
        # 持锁覆盖“检查 -> 调用 ROS -> 保存结果”，避免并发 HTTP 请求重复下单。
        with self.server.idempotency_lock:
            old = self.server.idempotency_store.get(key)
            if old is not None:
                if old["fingerprint"] != fingerprint:
                    return {
                        "success": False,
                        "error_code": "IDEMPOTENCY_KEY_REUSED",
                        "message": "Idempotency-Key was used with a different request",
                        "status_code": 409,
                    }
                replay = dict(old["response"])
                replay["idempotent_replay"] = True
                return replay

            result = self.server.submit_client.submit(payload)
            # 只缓存已经得到明确结果的请求；网关故障响应可以由客户端重新尝试。
            if result.get("status_code", 200) < 500:
                if len(self.server.idempotency_store) >= self.server.idempotency_limit:
                    self.server.idempotency_store.popitem(last=False)
                self.server.idempotency_store[key] = {
                    "fingerprint": fingerprint,
                    "response": dict(result),
                }
            return result

    def do_OPTIONS(self) -> None:
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, Idempotency-Key")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/health":
            self.send_json(200, {"success": True, "message": "rest gateway is running"})
            return
        if parsed.path == "/mission_events":
            query = parse_qs(parsed.query)
            mission_id = query.get("mission_id", [""])[0]
            result = self.server.submit_client.events(mission_id)
            status_code = int(result.pop("status_code", 200))
            self.send_json(status_code, result)
            return
        if parsed.path == "/mission_status":
            # 查询单个任务最新状态：GET /mission_status?mission_id=xxx
            # 任务不存在时仍返回 200，由响应里的 found=false / error_code 表达。
            query = parse_qs(parsed.query)
            mission_id = query.get("mission_id", [""])[0]
            result = self.server.submit_client.status(mission_id)
            status_code = int(result.pop("status_code", 200))
            self.send_json(status_code, result)
            return
        self.send_json(404, {"success": False, "message": "not found"})

    def do_POST(self) -> None:
        if self.path == "/dispatch_next":
            result = self.server.submit_client.trigger()
            status_code = int(result.pop("status_code", 200))
            self.send_json(status_code, result)
            return
        if self.path == "/cancel_mission":
            # 取消任务：可选 JSON body {"mission_id": "..."}。
            # 不传 body（或 body 为空 / mission_id 为空）时保留旧行为：
            # 取消当前活动任务，因此旧客户端的裸 POST 请求不受影响。
            mission_id = ""
            content_length = int(self.headers.get("Content-Length", "0"))
            if content_length > 0:
                body_text = self.rfile.read(content_length).decode("utf-8")
                if body_text.strip():
                    try:
                        body = json.loads(body_text)
                    except json.JSONDecodeError as error:
                        # 请求体不是合法 JSON 属于客户端错误，返回 400。
                        self.send_json(400, {"success": False, "message": str(error)})
                        return
                    if not isinstance(body, dict):
                        self.send_json(400, {"success": False, "message": "request body must be a JSON object"})
                        return
                    mission_id = str(body.get("mission_id", ""))
            result = self.server.submit_client.cancel(mission_id)
            status_code = int(result.pop("status_code", 200))
            self.send_json(status_code, result)
            return
        if self.path != "/v2/submit_order":
            self.send_json(404, {"success": False, "message": "not found"})
            return

        try:
            content_length = int(self.headers.get("Content-Length", "0"))
            if content_length <= 0:
                raise ValueError("request body is empty")
            payload = json.loads(self.rfile.read(content_length).decode("utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("request body must be a JSON object")
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
            self.send_json(400, {"success": False, "message": str(error)})
            return

        result = self._submit_with_idempotency(payload)
        status_code = int(result.pop("status_code", 200))
        self.send_json(status_code, result)


def main() -> None:
    parser = argparse.ArgumentParser(description="Minimal REST gateway for learn_order_node")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18080)
    parser.add_argument("--service", default="/v2/submit_order")
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    submit_client = SubmitOrderClient(args.service, args.timeout)
    server = ThreadingHTTPServer((args.host, args.port), RestHandler)
    server.submit_client = submit_client
    server.verbose = args.verbose
    # 幂等记录只保存在内存中，进程重启后清空；生产环境应放入持久化存储。
    server.idempotency_store = OrderedDict()
    server.idempotency_limit = 1024
    server.idempotency_lock = threading.Lock()
    print(f"REST gateway listening on http://{args.host}:{args.port}", flush=True)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        submit_client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
