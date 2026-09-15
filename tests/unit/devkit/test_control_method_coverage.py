"""Guard that the devkit only calls control-plane methods the runtime registers.

The devkit (`SimSession`) is a thin client over the JSON-RPC control plane. If it calls a
method the runtime never registers, the call fails against a real simulator with a confusing
"method not found" -- which is exactly what happened to `reset`, `enable_feature` and
`disable_feature`: client methods existed, the README showed them working, but no server
handler did. This test parses both sides statically (no Isaac Sim needed) and fails if they
ever drift apart again.
"""

from __future__ import annotations

from pathlib import Path
import re

REPO_ROOT = Path(__file__).resolve().parents[3]
RUNTIME = REPO_ROOT / "src" / "isaac_core" / "sim" / "runtime.py"
SESSION = REPO_ROOT / "src" / "isaac_core" / "devkit" / "session.py"
MESSAGES = REPO_ROOT / "src" / "isaac_core" / "control" / "messages.py"


def _registered_methods() -> set[str]:
    """Return the method names the runtime registers on its control server."""
    text = RUNTIME.read_text(encoding="utf-8")
    return set(re.findall(r'_control_server\.register\("([a-z_]+)"', text))


def _method_enum() -> dict[str, str]:
    """Return the Method enum as ``{MEMBER: "wire_name"}``."""
    text = MESSAGES.read_text(encoding="utf-8")
    return dict(re.findall(r"([A-Z_]+)\s*=\s*\"([a-z_]+)\"", text))


def _devkit_called_methods() -> set[str]:
    """Return the wire method names the devkit invokes via ``Method.X.value``."""
    text = SESSION.read_text(encoding="utf-8")
    enum = _method_enum()
    members = set(re.findall(r"Method\.([A-Z_]+)\.value", text))
    return {enum[m] for m in members if m in enum}


def test_devkit_only_calls_registered_control_methods() -> None:
    called = _devkit_called_methods()
    registered = _registered_methods()
    missing = called - registered
    assert not missing, (
        "the devkit calls these control methods that the runtime does not register, so they "
        f"fail with 'method not found' against a real simulator: {sorted(missing)}. Register a "
        "handler in sim/runtime.py (it may raise NotImplementedError with a clear message, but "
        "it must exist)."
    )


def test_the_devkit_actually_calls_something() -> None:
    # Guard against the parser silently matching nothing and passing vacuously.
    assert len(_devkit_called_methods()) >= 8


def test_runtime_registers_the_core_read_methods() -> None:
    # These must be real, working handlers, not just registered: they are the ones the
    # inspector and README quick-start depend on.
    registered = _registered_methods()
    for method in ("ping", "get_pose", "get_state", "get_capabilities", "get_config", "pause", "resume", "step"):
        assert method in registered, f"{method} must be registered"
