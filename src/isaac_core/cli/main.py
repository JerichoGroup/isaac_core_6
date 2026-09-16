"""Entry point for the ``isaac-core`` command-line tool.

Subcommands:

``run``
    Build the configuration and print what *would* be launched. The simulator
    runtime is not yet implemented.
``doctor``
    Run environment diagnostics.
``config dump``
    Print the fully-resolved configuration as TOML.
``config explain <key>``
    Show a specific key's winning value and source.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Final

if TYPE_CHECKING:
    from isaac_core.config import IsaacCoreConfig
    from isaac_core.install import IsaacInstall

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
from typing import Sequence

from isaac_core.cli.completion import register_completion_subcommand, run_completion
from isaac_core.cli.config_cmd import register_config_subcommand, run_config_dump, run_config_explain
from isaac_core.cli.doctor import run_doctor


def _build_parser() -> argparse.ArgumentParser:
    """Construct the argument parser with all subcommands.

    Returns:
        The fully configured :class:`argparse.ArgumentParser`.

    """
    parser = argparse.ArgumentParser(
        prog="isaac-core",
        description="Team infrastructure for Isaac Sim 6 simulations.",
    )
    subparsers = parser.add_subparsers(dest="command")

    # --- run ---
    run_parser = subparsers.add_parser("run", help="Launch the simulation")
    run_parser.add_argument("--config", type=str, default=None, help="Path to configuration TOML file")
    run_parser.add_argument("--isaac-path", type=str, default=None, help="Explicit path to Isaac Sim installation")
    run_parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Resolve everything and report what would launch, without starting Isaac Sim",
    )
    run_parser.add_argument(
        "--set",
        nargs=2,
        metavar=("KEY", "VALUE"),
        action="append",
        default=[],
        help="Override a config key (e.g. --set sim.headless true)",
    )

    # --- doctor ---
    subparsers.add_parser("doctor", help="Run environment diagnostics")

    # --- config ---
    register_config_subcommand(subparsers)

    # --- completion ---
    register_completion_subcommand(subparsers)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Parse arguments and dispatch to the appropriate subcommand.

    Args:
        argv: Command-line arguments. Defaults to ``sys.argv[1:]``.

    Returns:
        Exit code (0 = success, non-zero = failure).

    """
    parser = _build_parser()
    args = parser.parse_args(argv)

    if args.command is None:
        parser.print_help()
        return 0

    if args.command == "run":
        return _run_command(args)
    if args.command == "doctor":
        return run_doctor()
    if args.command == "config":
        return _dispatch_config(args)
    if args.command == "completion":
        return run_completion(args)

    parser.print_help()
    return 1


def _warn_about_an_unloaded_config_file() -> None:
    """Say so when a config file is sitting right there and is not being used.

    ``config/default.toml`` is a documented reference whose values match the schema defaults, and it
    is deliberately not auto-loaded: which file won would otherwise depend on the working directory.
    That is defensible and still catches people out, because editing it and seeing no change is
    silent and the only clue is the word "(defaults)" on the line above. Naming the file being
    ignored costs one line and removes the trap.

    """
    for candidate in (Path("config/default.toml"), Path("default.toml")):
        if candidate.is_file():
            print(f"           note: {candidate} exists but is NOT loaded automatically.")
            print(f"                 Pass it explicitly: --config {candidate}")
            return


def _run_command(args: argparse.Namespace) -> int:
    """Handle the ``run`` subcommand.

    Build and print the resolved configuration, then exit with an explicit message
    that the simulator runtime is not yet implemented.

    Args:
        args: Parsed arguments.

    Returns:
        Exit code.

    """

    from isaac_core.config import load
    from isaac_core.install import IsaacInstall, IsaacInstallError

    config_path = Path(args.config) if args.config else None
    cli_overrides: dict[str, str] = {}
    for key, value in args.set:
        cli_overrides[key] = value

    try:
        config = load(path=config_path, cli_overrides=cli_overrides if cli_overrides else None)
    except (FileNotFoundError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    isaac_path_arg = Path(args.isaac_path) if args.isaac_path else None
    try:
        install = IsaacInstall.locate(explicit_path=isaac_path_arg)
    except IsaacInstallError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print("=== isaac-core run ===")
    print(f"Isaac Sim: {install.root} (version {install.version})")
    if not install.is_supported():
        print(f"WARNING: {install.support_message}")
    print(f"Launcher:  {install.launcher_path}")
    print(f"Python:    {install.python_path}")
    env_config = os.environ.get("ISAAC_CORE_CONFIG")
    effective_config = config_path or (Path(env_config) if env_config else None)
    print(f"Config:    {effective_config or '(defaults)'}")
    if effective_config is None:
        _warn_about_an_unloaded_config_file()
    print(f"Headless:  {config.sim.headless}")
    print()

    if args.dry_run:
        print("Dry run: not launching.")
        return 0

    return _launch_simulator(install, config=config)


# Isaac ignores SIGTERM, so a polite request is only worth a short wait before SIGKILL.
_SIGTERM_GRACE_S: Final = 5.0


def _terminate_process_group(process: subprocess.Popen[bytes]) -> None:
    """Stop the simulator and everything it spawned, and do not return until it is gone.

    Isaac Sim starts Kit with ``installSignalHandlers=0`` and ignores ``SIGTERM``, and it spawns
    children of its own, so terminating just the direct child by just one signal leaves processes
    holding the GPU, the RTSP port and the UDP port. The process is its own session leader, so its
    group id is its pid and one call reaches the whole tree.

    Args:
        process: The simulator process, started with ``start_new_session=True``.

    """
    if process.poll() is not None:
        return
    try:
        group = os.getpgid(process.pid)
    except ProcessLookupError:
        return
    for signal_number in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(group, signal_number)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=_SIGTERM_GRACE_S if signal_number == signal.SIGTERM else _SIGTERM_GRACE_S)
            return
        except subprocess.TimeoutExpired:
            continue


def _launch_simulator(install: IsaacInstall, *, config: IsaacCoreConfig) -> int:
    """Launch the simulator in Isaac Sim's bundled interpreter.

    The runtime cannot run in this process: it needs ``omni``/``carb``/``pxr``, which
    exist only inside Isaac Sim's Python 3.12. So we exec ``-m isaac_core.sim`` there,
    which is why the package must be installed into that interpreter with
    ``python.sh -m pip install -e ".[sim]"``. Note this passes a *module*, never a
    filesystem path into this repo -- the install-not-locate rule from decision D7.

    The *fully resolved* config is written to a temporary TOML file and forwarded with
    ``--config``, rather than translating each override into a flag. That way env vars,
    CLI overrides and file settings all reach the simulator exactly as resolved here,
    and there is only one place that understands precedence.

    Args:
        install: The resolved :class:`~isaac_core.install.IsaacInstall`.
        config: The resolved configuration to hand to the simulator.

    Returns:
        The simulator's exit code, or 130 on keyboard interrupt.

    """

    from isaac_core.config.loader import dump_toml

    resolved_dir = Path(tempfile.mkdtemp(prefix="isaac-core-"))
    resolved_path = resolved_dir / "resolved.toml"
    resolved_path.write_text(dump_toml(config), encoding="utf-8")

    command = [str(install.python_path), "-m", "isaac_core.sim", "--config", str(resolved_path)]

    print("launching:", " ".join(command))
    print(f"resolved config: {resolved_path}")
    print()
    try:
        # Its own session, so Ctrl-C reaches this process only and stopping the simulator is
        # unambiguously our job. Sharing the terminal's process group meant SIGINT arrived while Kit
        # was still starting up, Kit carried on regardless, and returning 130 here left it running --
        # it would finish loading minutes later and pop a window belonging to a command that had
        # already exited.
        process = subprocess.Popen(command, start_new_session=True)
    except OSError as exc:
        print(f"ERROR: could not launch the simulator: {exc}", file=sys.stderr)
        return 1

    try:
        return process.wait()
    except KeyboardInterrupt:
        print("\ninterrupted; stopping the simulator")
        _terminate_process_group(process)
        return 130


def _dispatch_config(args: argparse.Namespace) -> int:
    """Dispatch ``config`` sub-subcommands.

    Args:
        args: Parsed arguments.

    Returns:
        Exit code.

    """
    config_subcmd = getattr(args, "config_command", None)
    if config_subcmd == "dump":
        return run_config_dump(config_path=args.config)
    if config_subcmd == "explain":
        return run_config_explain(key=args.key, config_path=args.config)
    # No sub-subcommand: print config help
    print("usage: isaac-core config {dump,explain} ...", file=sys.stderr)
    return 1


def cli() -> None:
    """Console-script entry point.

    Calls :func:`main` and exits with its return code.
    """
    sys.exit(main())


if __name__ == "__main__":
    cli()
