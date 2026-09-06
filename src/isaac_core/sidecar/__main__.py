"""
Entry point for the sidecar service host.

Usage::

    python3 -m isaac_core.sidecar --config <path>

Runs the :class:`~isaac_core.sidecar.service.ServiceSupervisor` until interrupted
(SIGINT / Ctrl-C), then shuts down all services gracefully.
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path
import signal
import sys
import threading

from isaac_core.config import load
from isaac_core.sidecar.service import RestartPolicy, ServiceSupervisor

logger = logging.getLogger("isaac_core.sidecar")


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """
    Parse command-line arguments.

    Args:
        argv: Argument list (defaults to ``sys.argv[1:]``).

    Returns:
        Parsed namespace with ``config`` path.

    """
    parser = argparse.ArgumentParser(
        prog="python3 -m isaac_core.sidecar",
        description="Isaac Core sidecar service host",
    )
    parser.add_argument(
        "--config",
        type=Path,
        required=True,
        help="Path to the TOML configuration file",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """
    Run the sidecar supervisor until interrupted.

    Args:
        argv: Argument list (defaults to ``sys.argv[1:]``).

    Returns:
        Exit code (0 on clean shutdown, 1 on error).

    """
    args = _parse_args(argv)

    config = load(path=args.config)

    log_level = config.logging.level.upper()
    logging.basicConfig(
        level=getattr(logging, log_level, logging.INFO),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    sidecar_config = config.sidecar
    if not sidecar_config.enabled:
        logger.info("Sidecar is not enabled in config; exiting")
        return 0

    if not sidecar_config.services:
        logger.info("No services configured; exiting")
        return 0

    policy = RestartPolicy()
    supervisor = ServiceSupervisor.from_config(sidecar_config, policy=policy)

    if not supervisor.services:
        logger.error("No services could be instantiated; exiting")
        return 1

    shutdown_event = _setup_signal_handling()

    supervisor.start()
    logger.info("Sidecar supervisor started with %d service(s)", len(supervisor.services))

    try:
        shutdown_event.wait()
    except KeyboardInterrupt:
        pass

    logger.info("Shutting down sidecar supervisor")
    supervisor.shutdown()
    return 0


def _setup_signal_handling() -> threading.Event:
    """
    Install SIGINT/SIGTERM handlers that set an event.

    Returns:
        An event that is set when a termination signal is received.

    """
    event = threading.Event()

    def _handler(signum: int, frame: object) -> None:  # noqa: ARG001
        event.set()

    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)
    return event


if __name__ == "__main__":
    sys.exit(main())
