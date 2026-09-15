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

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isaac_core.config import IsaacCoreConfig
    from isaac_core.install import IsaacInstall

import argparse
from pathlib import Path
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
    print(f"Config:    {config_path or '(defaults)'}")
    print(f"Headless:  {config.sim.headless}")
    print()

    if args.dry_run:
        print("Dry run: not launching.")
        return 0

    return _launch_simulator(install, config=config)


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
        return subprocess.call(command)
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 130
    except OSError as exc:
        print(f"ERROR: could not launch the simulator: {exc}", file=sys.stderr)
        return 1


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
