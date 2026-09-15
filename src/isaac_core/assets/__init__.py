"""Shipped assets: scenes and feature-layer manifests bundled in the wheel.

The package exposes :data:`LAYERS_DIR` for programmatic access to the shipped
layer manifests, which discovery can scan alongside user-provided search paths.
"""

from pathlib import Path
from typing import Final

# Root of the shipped layer manifests, each in its own subdirectory with a layer.toml.
LAYERS_DIR: Final = Path(__file__).parent / "layers"

__all__ = ["LAYERS_DIR"]
