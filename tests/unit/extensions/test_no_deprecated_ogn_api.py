"""
Guard against deprecated OmniGraph APIs re-entering the Kit extensions.

Isaac Sim 6 / Kit 110 deprecated the state *accessors* the previous generation used.
Read from ``omni/graph/core/_impl/database.py`` in omni.graph 1.142.5:

===================================== ==========================================
Deprecated                            Current
===================================== ==========================================
``db.internal_state``                 ``db.per_instance_state``
``Db.per_node_internal_state(node)``  ``Db.per_instance_internal_state(node)``
===================================== ==========================================

The deprecation is not fatal yet -- it warns on every compute, which is noisy enough
to bury real errors, and will eventually be removed.

Note what is **not** deprecated: the static ``internal_state()`` factory on the node
class. That is the OGN hook which constructs the state object, and Isaac's own
``OgnROS2CameraHelper`` still declares it. Only the accessors changed. Do not "fix"
the factory.

These tests read source as text, so they run with neither Isaac Sim nor ROS 2 present
-- which matters, because this deprecation is otherwise only visible in a Kit console
at runtime.
"""

from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
EXTENSIONS_ROOT = REPO_ROOT / "extensions"

NODE_FILES = sorted(EXTENSIONS_ROOT.rglob("*/nodes/*.py"))

# Accessor -> replacement. Each is a plain substring; these names are distinctive
# enough that a substring match will not produce false positives.
DEPRECATED_ACCESSORS = {
    "db.internal_state": "db.per_instance_state",
    "per_node_internal_state": "per_instance_internal_state",
}


def test_node_files_were_discovered() -> None:
    # Guard against the glob matching nothing and the tests below passing vacuously.
    assert NODE_FILES, f"no node modules found under {EXTENSIONS_ROOT}"


@pytest.mark.parametrize("deprecated", sorted(DEPRECATED_ACCESSORS), ids=lambda s: s)
def test_no_node_uses_a_deprecated_state_accessor(deprecated: str) -> None:
    replacement = DEPRECATED_ACCESSORS[deprecated]
    offenders = [
        str(path.relative_to(REPO_ROOT)) for path in NODE_FILES if deprecated in path.read_text(encoding="utf-8")
    ]
    assert not offenders, (
        f"{deprecated!r} is deprecated in Kit 110 and warns on every compute. "
        f"Use {replacement!r} instead.\n  " + "\n  ".join(offenders)
    )


def test_stateful_nodes_still_declare_the_state_factory() -> None:
    # The static internal_state() factory is NOT deprecated -- it is the OGN hook that
    # builds the state object. Removing it would leave per_instance_state empty, which
    # fails at runtime rather than at lint time, so it is worth pinning.
    for path in NODE_FILES:
        source = path.read_text(encoding="utf-8")
        if "per_instance_state" in source or "per_instance_internal_state" in source:
            assert "def internal_state(" in source, (
                f"{path.name} reads per-instance state but declares no " f"internal_state() factory to create it"
            )


def test_release_hooks_use_the_current_accessor() -> None:
    # release() looks state up by node rather than through a db, so it needs the
    # classmethod form. Getting this wrong only surfaces during teardown.
    for path in NODE_FILES:
        source = path.read_text(encoding="utf-8")
        if "def release(" in source and "internal_state" in source:
            assert (
                "per_instance_internal_state(" in source
            ), f"{path.name}: release() should use per_instance_internal_state(node)"
