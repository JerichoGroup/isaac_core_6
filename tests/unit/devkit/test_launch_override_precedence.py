"""`Sim.launch` must not let a defaulted argument clobber an explicit override.

The README documents pointing at your own scene like this:

```python
Sim.launch(overrides={"sim.scene": "/home/you/my_project/terrain.usda"})
```

That silently loaded the shipped `earth` scene instead. `launch` applied its explicit arguments last so
`headless=True` could not be contradicted, but `scene` defaulted to `"earth"`, so "not passed" and
"passed the default" were indistinguishable and the default won. Found live: the state reported
`scene: 'earth'` while recording what was supposed to be a different stage.

The fix keeps explicit-wins and adds absent-loses, which needs both directions tested.
"""

from __future__ import annotations

from typing import Any

from isaac_core.devkit import Sim


class _Captured:
    """A launcher that records the config it was asked to run with, and never starts anything."""

    def __init__(self) -> None:
        self.commands: list[list[str]] = []

    def __call__(self, command: list[str]) -> Any:
        self.commands.append(command)
        raise _StopError


class _StopError(Exception):
    """Raised to end `launch` once the config has been written."""


def _config_for(**kwargs: Any) -> dict[str, Any]:
    """Return the resolved config `launch` would have run with.

    Args:
        **kwargs: Passed to `Sim.launch`.

    Returns:
        The parsed config file `launch` wrote for the simulator.

    """
    import json
    from pathlib import Path
    import re

    launcher = _Captured()
    try:
        Sim.launch(launcher=launcher, **kwargs)
    except _StopError:
        pass
    except Exception:  # resolution may fail after the config is written; that is fine
        pass
    assert launcher.commands, "launch never reached the launcher"
    command = launcher.commands[0]
    match = [part for part in command if part.endswith(".json") or part.endswith(".toml")]
    assert match, f"no config file in the command: {command}"
    text = Path(match[-1]).read_text(encoding="utf-8")
    if match[-1].endswith(".json"):
        parsed: dict[str, Any] = json.loads(text)
        return parsed
    # A minimal TOML reader is not needed: pull the two keys under test out directly.
    scene = re.search(r'^\s*scene\s*=\s*"([^"]+)"', text, re.M)
    headless = re.search(r"^\s*headless\s*=\s*(true|false)", text, re.M)
    return {
        "sim": {
            "scene": scene.group(1) if scene else None,
            "headless": headless.group(1) == "true" if headless else None,
        }
    }


def test_an_override_sets_the_scene_when_the_argument_is_absent() -> None:
    # The README's documented way to load your own scene.
    config = _config_for(overrides={"sim.scene": "/tmp/my_project/terrain.usda"})
    assert config["sim"]["scene"] == "/tmp/my_project/terrain.usda"


def test_an_explicit_scene_still_wins_over_an_override() -> None:
    # Explicit-wins is the property that stops an override quietly contradicting a caller.
    config = _config_for(scene="earth", overrides={"sim.scene": "/tmp/other.usda"})
    assert config["sim"]["scene"] == "earth"


def test_the_shipped_scene_is_still_the_default() -> None:
    config = _config_for()
    assert config["sim"]["scene"] == "earth"


def test_an_override_sets_headless_when_the_argument_is_absent() -> None:
    config = _config_for(overrides={"sim.headless": True})
    assert config["sim"]["headless"] is True


def test_an_explicit_headless_still_wins_over_an_override() -> None:
    config = _config_for(headless=False, overrides={"sim.headless": True})
    assert config["sim"]["headless"] is False


def test_headless_defaults_to_a_window() -> None:
    config = _config_for()
    assert config["sim"]["headless"] is False
