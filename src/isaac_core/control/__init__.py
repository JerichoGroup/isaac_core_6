"""JSON-RPC control plane for commanding a running Isaac Sim session.

Architecture note: docs/development-log.md §7 placed the ControlPlane conceptually inside ``sim``,
but it lives in its own package because the CLIENT must be importable by ``devkit``
on a laptop that has no Isaac Sim, while the SERVER runs inside Isaac's interpreter.
A shared, dependency-free package is the only way both sides can use identical
message definitions — the same reasoning that produced ``contracts``.

Without a control plane every setting would be a launch flag, so changing a
camera resolution meant killing the simulator, editing ``consts.py`` and relaunching.
Readiness was detected by grepping stdout for ``"rclpy loaded"`` followed by an
unconditional ``sleep(5)``. This package retires both problems: it is JSON-RPC over a
local socket, and the port becoming connectable IS the readiness signal.
"""

from isaac_core.control.client import ControlClient
from isaac_core.control.errors import (
    AuthError,
    InternalError,
    InvalidParamsError,
    InvalidRequestError,
    MethodNotFoundError,
    ParseError,
    RpcError,
)
from isaac_core.control.messages import Method, RpcErrorData, RpcRequest, RpcResponse
from isaac_core.control.server import ControlServer

__all__ = [
    "AuthError",
    "ControlClient",
    "ControlServer",
    "InternalError",
    "InvalidParamsError",
    "InvalidRequestError",
    "Method",
    "MethodNotFoundError",
    "ParseError",
    "RpcError",
    "RpcErrorData",
    "RpcRequest",
    "RpcResponse",
]
