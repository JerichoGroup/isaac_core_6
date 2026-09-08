"""
Guard the control plane's method surface end to end.

The existing coverage guard (tests/unit/devkit/test_control_method_coverage.py) only checks
the methods the devkit *happens* to call. That leaves two gaps this file closes:

1. A ``Method`` enum member with no registered handler. Nothing in the devkit needs to call it
   for it to be a live part of the wire contract -- an operator poking the control plane by hand
   (or a future client) hits "method not found", which reads like a client/server version
   mismatch rather than "this was never wired up".
2. A registered handler that nothing can reach. A handler no devkit method, CLI, or shipped debug
   tool ever calls is dead surface: it can never be invoked, so it can rot silently.

Everything here is parsed statically from source, so the suite runs with no Isaac Sim, no ROS 2
and no running control plane.
"""

from __future__ import annotations

from pathlib import Path
import re
from typing import Final

REPO_ROOT = Path(__file__).resolve().parents[3]
RUNTIME = REPO_ROOT / "src" / "isaac_core" / "sim" / "runtime.py"
MESSAGES = REPO_ROOT / "src" / "isaac_core" / "control" / "messages.py"

# Source trees that may legitimately call a control method: the control client itself (ping is
# the readiness probe), the devkit client, the CLI, and the terminal debug tools
# (isaac-core-inspect calls get_runtime_values, for example).
_CALLER_ROOTS: Final = (
    REPO_ROOT / "src" / "isaac_core" / "control",
    REPO_ROOT / "src" / "isaac_core" / "devkit",
    REPO_ROOT / "src" / "isaac_core" / "cli",
    REPO_ROOT / "src" / "isaac_core" / "debug",
)

# Enum members that are deliberately allowed to have no registered handler. Empty by design:
# every wire method the enum advertises must be answerable by the runtime, even if the handler
# only raises a clear NotImplementedError. Grow this only with a written reason in the audit.
_ENUM_WITHOUT_HANDLER_OK: Final[frozenset[str]] = frozenset()

# Registered handlers that are allowed to have no external caller. These are internal-only
# methods invoked by name (not via the Method enum) from a shipped tool. Each entry names the
# tool that reaches it, so an entry that stops being true will be caught by the reachability test
# below rather than hidden here.
#   get_runtime_values  -> isaac-core-inspect (src/isaac_core/debug/inspector.py)
_INTERNAL_HANDLERS_OK: Final[frozenset[str]] = frozenset({"get_runtime_values"})


def _registered_methods() -> set[str]:
    """Return the wire names the runtime registers on its control server."""
    text = RUNTIME.read_text(encoding="utf-8")
    return set(re.findall(r'_control_server\.register\("([a-z_]+)"', text))


def _enum_wire_names() -> set[str]:
    """Return the wire values of every Method enum member."""
    text = MESSAGES.read_text(encoding="utf-8")
    # Only the Method enum uses the ``MEMBER = "wire_name"`` form in this module.
    return {wire for _member, wire in re.findall(r'([A-Z_]+)\s*=\s*"([a-z_]+)"', text)}


def _called_wire_names() -> set[str]:
    """
    Return every control-method wire name any caller tree invokes.

    Matches both spellings a caller can use: ``client.call("get_state")`` with a string literal,
    and ``Method.GET_STATE.value`` via the enum (resolved back to its wire name).

    Returns:
        The set of wire names reachable from the devkit, CLI or debug tools.

    """
    enum = {member: wire for member, wire in re.findall(r'([A-Z_]+)\s*=\s*"([a-z_]+)"', MESSAGES.read_text("utf-8"))}
    called: set[str] = set()
    for root in _CALLER_ROOTS:
        for path in root.rglob("*.py"):
            text = path.read_text(encoding="utf-8")
            called.update(re.findall(r'\.call\(\s*"([a-z_]+)"', text))
            called.update(enum[m] for m in re.findall(r"Method\.([A-Z_]+)\.value", text) if m in enum)
    return called


def test_the_parsers_found_something() -> None:
    # Without this, every assertion below could pass vacuously if a regex silently stopped
    # matching (e.g. the register(...) call style changed).
    assert len(_registered_methods()) >= 8, "runtime registration parser matched too little"
    assert len(_enum_wire_names()) >= 8, "Method enum parser matched too little"
    assert len(_called_wire_names()) >= 8, "caller parser matched too little"


def test_every_enum_method_has_a_registered_handler() -> None:
    # Failure mode: a Method enum member with no runtime handler. Calling it fails with
    # "method not found", which an operator reads as a version mismatch, not "never wired up".
    # A handler may raise NotImplementedError, but it must be registered so the error is honest.
    enum = _enum_wire_names()
    registered = _registered_methods()
    missing = enum - registered - _ENUM_WITHOUT_HANDLER_OK
    assert not missing, (
        "these Method enum members have no registered handler in sim/runtime.py, so they fail "
        f"with 'method not found' rather than an honest error: {sorted(missing)}. Register a "
        "handler (it may raise NotImplementedError with a clear message)."
    )


def test_every_registered_handler_is_reachable() -> None:
    # Failure mode: a handler nothing can call is dead control surface -- it can never be
    # invoked, so a bug in it is invisible and the code rots. A handler counts as reachable if it
    # is a published Method enum member (an operator can call it by hand, and test_every_enum_
    # method_has_a_registered_handler keeps the enum honest), OR a caller tree (control client,
    # devkit, CLI, debug) invokes it by string or Method.X.value, OR it is an explicitly
    # documented internal handler in the allowlist above. Anything else is registered but
    # unreachable: not in the contract, not called, not a known internal.
    registered = _registered_methods()
    reachable = _enum_wire_names() | _called_wire_names() | _INTERNAL_HANDLERS_OK
    unreachable = registered - reachable
    assert not unreachable, (
        "these control methods are registered but are neither in the Method enum nor called by "
        f"anything (control client, devkit, CLI or debug tool), so they are dead surface: "
        f"{sorted(unreachable)}. Either add them to the Method enum and give them a caller, or "
        "remove the registration and handler, or -- if a shipped tool calls it by name -- add it "
        "to _INTERNAL_HANDLERS_OK with the tool named."
    )
