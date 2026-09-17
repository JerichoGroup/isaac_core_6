"""Locate and validate an Isaac Sim installation.

The resolution order deliberately ignores every environment variable -- such a variable
is stale on many team machines, pointing at the old 2023.1.1 install while the
real Isaac Sim 6.x lives elsewhere.

Order: explicit path -> config -> probe a list of known candidate locations.
"Validates" means the directory actually contains ``python.sh``, ``isaac-sim.sh``
and a ``VERSION`` file.
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

# Required marker files -- if any is missing the directory is not a valid install.
_REQUIRED_FILES: tuple[str, ...] = ("python.sh", "isaac-sim.sh", "VERSION")

# Known candidate locations to probe, in priority order.
# Locations probed when nothing more specific is configured. Home-relative entries
# are resolved at call time so this works for every team member, not just whoever
# happened to install first -- a hardcoded /home/<someone>/isaacsim would silently
# fail for everyone else.
_HOME_RELATIVE_CANDIDATES: tuple[str, ...] = (
    "isaacsim",
    ".local/share/ov/pkg/isaacsim",
)

_ABSOLUTE_CANDIDATES: tuple[str, ...] = (
    "/opt/isaacsim",
    "/opt/nvidia/isaac-sim",
    "/isaac-sim",
)

# Environment variable that might point at the install (often stale).

# Major version required for the new extension namespace (isaacsim.* vs omni.isaac.*).
_MINIMUM_SUPPORTED_MAJOR = 6

# Year-based versions (2022, 2023, etc.) predate the 6.x numbering scheme.
_YEAR_VERSION_THRESHOLD = 100


def _probe_candidates() -> tuple[str, ...]:
    """Return the directories probed when no install is explicitly configured.

    Home-relative entries are resolved here rather than at import time, so the list
    is correct for whichever user is running it and tests have a single seam to
    patch instead of two separate constants.

    Returns:
        Absolute candidate paths, most likely first.

    """
    home = [str(Path.home() / rel) for rel in _HOME_RELATIVE_CANDIDATES]
    return tuple(home) + _ABSOLUTE_CANDIDATES


class IsaacInstallError(RuntimeError):
    """Raise when no valid Isaac Sim installation can be found.

    The message includes all locations that were tried, so the user knows what to
    fix.
    """


class IsaacInstall:
    """Represent a validated Isaac Sim installation on disk.

    Exposes the version, key paths, and a support check. Instantiate via
    :meth:`locate` rather than calling the constructor directly with an unvalidated
    path.
    """

    def __init__(self, root: Path) -> None:
        """Wrap a *pre-validated* root path.

        Args:
            root: Directory that has already been confirmed to contain the required
                marker files.

        """
        self._root = root
        self._version = (root / "VERSION").read_text(encoding="utf-8").strip()

    @property
    def root(self) -> Path:
        """Return the installation root directory."""
        return self._root

    @property
    def version(self) -> str:
        """Return the full version string from the VERSION file."""
        return self._version

    @property
    def major_version(self) -> int:
        """Return the major version number.

        Returns:
            Integer major version. Returns ``0`` if parsing fails.

        """
        try:
            return int(self._version.split(".")[0])
        except (ValueError, IndexError):
            return 0

    @property
    def python_path(self) -> Path:
        """Return the path to Isaac's bundled ``python.sh`` launcher."""
        return self._root / "python.sh"

    @property
    def launcher_path(self) -> Path:
        """Return the path to ``isaac-sim.sh``."""
        return self._root / "isaac-sim.sh"

    @property
    def exts_user_dir(self) -> Path:
        """Return the ``extsUser`` directory where user extensions are symlinked."""
        return self._root / "extsUser"

    def is_supported(self) -> bool:
        """Return whether this install is Isaac Sim 6.x or newer.

        Versions older than 6.0 used the ``omni.isaac.*`` extension namespace, which
        was renamed to ``isaacsim.*`` in 4.5. Older installs will not work with our
        extensions. Year-based versions (e.g. 2023.1.1) predate the 6.x numbering
        and are not supported.

        Returns:
            ``True`` if version >= 6.x and not a year-based version.

        """
        major = self.major_version
        # Year-based versions (2022, 2023, ...) used the old namespace
        if major >= _YEAR_VERSION_THRESHOLD:
            return False
        return major >= _MINIMUM_SUPPORTED_MAJOR

    @property
    def support_message(self) -> str:
        """Return a human-readable support status string.

        Returns:
            A message indicating whether the install is supported or explaining why
            not.

        """
        if self.is_supported():
            return f"Isaac Sim {self._version} is supported"
        return (
            f"Isaac Sim {self._version} is NOT supported. "
            f"Version 6.x or newer is required. "
            f"Every omni.isaac.* extension was renamed to isaacsim.* in 4.5; "
            f"older installs will not work with our extensions."
        )

    @classmethod
    def locate(
        cls,
        explicit_path: Path | None = None,
        config_path: Path | None = None,
        environ: dict[str, str] | None = None,
        extra_candidates: Sequence[Path] | None = None,
    ) -> "IsaacInstall":
        """Find and validate an Isaac Sim installation.

        Resolution walks the sources below in order, but a candidate only wins
        outright if it is both structurally valid **and** a supported version. A
        structurally valid but unsupported install (say a leftover 2023.1.1) is
        remembered as a fallback and used only when nothing better is found.

        That distinction is the whole point of this function. On a machine that has been
        through several Isaac Sim versions, an old install passes every structural check
        while being useless -- every ``omni.isaac.*`` extension was renamed to
        ``isaacsim.*`` in 4.5, so our extensions cannot load there. Preferring the newest
        supported install means the tooling does the right thing on its own.

        No environment variable is consulted. A shell variable naming an install is a
        setting the user cannot see and usually cannot remember setting, and the value is
        stale more often than not. Where it is genuinely needed, an explicit
        ``--isaac-path`` says the same thing visibly and only for the command that needs it.

        Sources, in order:
            1. *explicit_path* argument
            2. *config_path* from the configuration file
            3. Probe a list of known candidate locations

        Args:
            explicit_path: Directly supplied path (e.g. from a CLI ``--isaac-path``
                flag).
            config_path: Path read from the configuration file's
                ``sim.isaac_sim_path``.
            environ: Accepted and ignored, so an existing caller keeps working. No
                environment variable takes part in resolution.
            extra_candidates: Additional directories to probe after the built-in
                candidates.

        Returns:
            A validated :class:`IsaacInstall`, preferring a supported version.

        Raises:
            IsaacInstallError: If no valid installation is found after trying all
                resolution steps.

        """

        tried: list[str] = []
        fallback: IsaacInstall | None = None

        def consider(path: Path, label: str) -> "IsaacInstall | None":
            """Return the install if it is supported, else record it as a fallback."""
            nonlocal fallback
            if not _is_valid_install(path):
                tried.append(f"{label}: {path}")
                return None
            candidate = cls(path)
            if candidate.is_supported():
                return candidate
            if fallback is None:
                fallback = candidate
            tried.append(f"{label}: {path} (found, but unsupported version {candidate.version})")
            return None

        if explicit_path is not None:
            # An explicit path is an instruction, not a guess: honour it even if the
            # version is unsupported, so a user can deliberately target an old install.
            if _is_valid_install(explicit_path):
                return cls(explicit_path)
            # But if it is invalid, fail here rather than silently probing and using a
            # *different* install than the one the user explicitly named -- that surprise
            # is worse than an error.
            msg = (
                f"--isaac-path (or explicit isaac_sim_path) {explicit_path} is not a valid "
                "Isaac Sim installation. A valid install contains: python.sh, isaac-sim.sh, "
                "VERSION. Fix the path, or omit it to auto-detect."
            )
            raise IsaacInstallError(msg)

        if config_path is not None:
            found = consider(config_path, "config")
            if found is not None:
                return found

        candidates = list(_probe_candidates())
        if extra_candidates:
            candidates.extend(str(p) for p in extra_candidates)
        for candidate_str in candidates:
            found = consider(Path(candidate_str), "probe")
            if found is not None:
                return found

        if fallback is not None:
            return fallback

        # Nothing found
        locations = "\n  ".join(tried)
        msg = (
            "Cannot find a valid Isaac Sim installation.\n"
            f"Tried (in order):\n  {locations}\n\n"
            "A valid install contains: python.sh, isaac-sim.sh, VERSION.\n"
            "Fix: set isaac_sim_path in your config file, or pass --isaac-path."
        )
        raise IsaacInstallError(msg)


def _is_valid_install(path: Path) -> bool:
    """Check whether a directory looks like a valid Isaac Sim installation.

    Args:
        path: Candidate directory.

    Returns:
        ``True`` if the directory exists and contains all required marker files.

    """
    if not path.is_dir():
        return False
    return all((path / name).is_file() for name in _REQUIRED_FILES)
