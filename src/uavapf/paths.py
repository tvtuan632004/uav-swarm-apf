"""Location of simulation output.

Results never go into the repository. Every writer asks :func:`results_root` for the
output root, which is controlled by the environment variable ``UAVAPF_RESULTS``
(point it at a large HDD). See docs/decisions.md, D001.
"""

from __future__ import annotations

import os
from pathlib import Path

ENV_VAR: str = "UAVAPF_RESULTS"


def results_root() -> Path:
    """Return the absolute output root for results.

    The environment is read at call time, so changing ``UAVAPF_RESULTS`` inside a running
    process takes effect immediately. The directory is *not* created; use
    :func:`ensure_dir` when you are about to write.

    Returns
    -------
    pathlib.Path
        ``$UAVAPF_RESULTS`` with ``~`` expanded and made absolute (relative values resolve
        against the current working directory). If the variable is unset or blank,
        ``<cwd>/results``.
    """
    value = os.environ.get(ENV_VAR, "").strip()
    root = Path(value).expanduser() if value else Path("results")
    return root.absolute()


def ensure_dir(path: Path) -> Path:
    """Create ``path`` (and parents) if missing and return it unchanged."""
    path.mkdir(parents=True, exist_ok=True)
    return path
