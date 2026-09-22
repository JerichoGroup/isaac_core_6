"""`config dump` and `config explain` must see `--set`, the same as `run` does.

The README lists five resolution sources and documents `--set` as the fourth. `config explain` exists to
say "which source won, and what the others offered". Neither config subcommand accepted `--set` at all, so
you could not preview what an override would do, and the command whose entire job is naming the winning
source could not report the source you were most likely debugging.

Found by running the CLI the way a user does rather than calling the loader directly.
"""

from __future__ import annotations

import pytest

from isaac_core.cli.config_cmd import overrides_from_args
from isaac_core.cli.main import main

SCENE = "/tmp/some_project/house.usda"


# -- the argument is accepted at all -------------------------------------------- #


def test_dump_accepts_set(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["config", "dump", "--set", "sim.scene", SCENE]) == 0
    assert SCENE in capsys.readouterr().out


def test_explain_accepts_set(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["config", "explain", "sim.scene", "--set", "sim.scene", SCENE]) == 0
    assert SCENE in capsys.readouterr().out


# -- and it actually changes the answer ----------------------------------------- #


def test_dump_reflects_the_override_rather_than_the_default(capsys: pytest.CaptureFixture[str]) -> None:
    main(["config", "dump"])
    default = capsys.readouterr().out
    main(["config", "dump", "--set", "sim.scene", SCENE])
    overridden = capsys.readouterr().out
    assert 'scene = "earth"' in default
    assert SCENE in overridden
    assert overridden != default


def test_explain_names_the_flag_as_the_winning_source(capsys: pytest.CaptureFixture[str]) -> None:
    # The point of the command. Reporting "default" while a --set was passed would be a wrong answer, not
    # a missing feature.
    main(["config", "explain", "sim.scene", "--set", "sim.scene", SCENE])
    out = capsys.readouterr().out
    assert "source: cli" in out, out


def test_explain_still_reports_default_without_the_flag(capsys: pytest.CaptureFixture[str]) -> None:
    main(["config", "explain", "sim.scene"])
    out = capsys.readouterr().out
    assert "source: default" in out, out
    assert "earth" in out


def test_several_overrides_all_apply(capsys: pytest.CaptureFixture[str]) -> None:
    code = main(
        [
            "config",
            "dump",
            "--set",
            "sim.scene",
            SCENE,
            "--set",
            "sim.headless",
            "true",
        ]
    )
    assert code == 0
    out = capsys.readouterr().out
    assert SCENE in out
    assert "headless = true" in out


def test_an_invalid_override_is_rejected_rather_than_ignored(capsys: pytest.CaptureFixture[str]) -> None:
    # Silently dropping an unparseable value would make dump lie about what would launch.
    assert main(["config", "dump", "--set", "sim.headless", "notabool"]) == 1
    assert "headless" in capsys.readouterr().err.lower()


def test_an_unknown_key_is_rejected(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["config", "dump", "--set", "sim.no_such_key", "1"]) == 1
    captured = capsys.readouterr()
    assert "no_such_key" in (captured.err + captured.out)


# -- the pair-to-mapping helper -------------------------------------------------- #


def test_no_pairs_means_none_not_an_empty_dict() -> None:
    # `load` distinguishes "no overrides" from "an empty override set", so this must not flatten them.
    assert overrides_from_args(None) is None
    assert overrides_from_args([]) is None


def test_pairs_become_a_mapping() -> None:
    assert overrides_from_args([["sim.scene", SCENE], ["sim.headless", "true"]]) == {
        "sim.scene": SCENE,
        "sim.headless": "true",
    }


def test_a_repeated_key_takes_the_last_value() -> None:
    # argparse appends, so the later flag has to win, matching how a shell user expects it to read.
    assert overrides_from_args([["sim.scene", "first"], ["sim.scene", "second"]]) == {"sim.scene": "second"}


def test_run_and_config_take_the_same_flag_shape() -> None:
    # Two spellings of the same flag would be its own bug. Asserted by parsing the same argument text
    # through both subcommands, which is what a user would type, rather than by introspecting argparse.
    from isaac_core.cli.main import _build_parser

    parser = _build_parser()
    run_args = parser.parse_args(["run", "--set", "sim.scene", "x.usda", "--set", "sim.headless", "true"])
    dump_args = parser.parse_args(["config", "dump", "--set", "sim.scene", "x.usda", "--set", "sim.headless", "true"])
    explain_args = parser.parse_args(["config", "explain", "sim.scene", "--set", "sim.scene", "x.usda"])

    assert run_args.set == [["sim.scene", "x.usda"], ["sim.headless", "true"]]
    assert dump_args.set == run_args.set, "config dump parses --set differently from run"
    assert explain_args.set == [["sim.scene", "x.usda"]]
