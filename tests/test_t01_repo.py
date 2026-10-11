"""T01 acceptance tests: repo skeleton, package layout, results path helper.

Spec: docs/specs/T01-repo-setup.md. Implementer must not edit this file.
"""

from __future__ import annotations

import importlib
import tomllib
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
SUBPACKAGES = ["sim", "geometry", "forces", "control", "baselines", "scenarios", "metrics"]


# --------------------------------------------------------------------------- #
# Repo files
# --------------------------------------------------------------------------- #
def test_python_version_pinned():
    assert (REPO / ".python-version").read_text(encoding="utf-8").strip() == "3.12"


def test_pyproject_basics():
    data = tomllib.loads((REPO / "pyproject.toml").read_text(encoding="utf-8"))
    project = data["project"]
    assert project["name"] == "uavapf"
    assert "3.12" in project["requires-python"]
    deps = " ".join(project["dependencies"]).lower()
    for pkg in ("numpy", "scipy", "pyyaml", "matplotlib"):
        assert pkg in deps, f"missing runtime dependency: {pkg}"
    dev = " ".join(data["dependency-groups"]["dev"]).lower()
    assert "pytest" in dev and "ruff" in dev
    assert data["tool"]["ruff"]["line-length"] == 100


def test_lockfile_committed():
    assert (REPO / "uv.lock").is_file()


def test_gitignore_excludes_results_and_venv():
    lines = {ln.strip() for ln in (REPO / ".gitignore").read_text(encoding="utf-8").splitlines()}
    assert "results/" in lines
    assert ".venv/" in lines


def test_ci_runs_on_both_oses():
    ci = (REPO / ".github" / "workflows" / "ci.yml").read_text(encoding="utf-8")
    for needle in ("windows-latest", "ubuntu-latest", "uv sync", "pytest", "ruff check"):
        assert needle in ci, f"ci.yml should mention {needle!r}"


@pytest.mark.parametrize("folder", ["configs", "scripts", "paper", "docs/specs"])
def test_top_level_folders_exist(folder):
    assert (REPO / folder).is_dir()


# --------------------------------------------------------------------------- #
# Package layout
# --------------------------------------------------------------------------- #
def test_package_version():
    import uavapf

    assert uavapf.__version__ == "0.1.0"


@pytest.mark.parametrize("name", SUBPACKAGES)
def test_subpackages_importable(name):
    importlib.import_module(f"uavapf.{name}")


# --------------------------------------------------------------------------- #
# uavapf.paths
# --------------------------------------------------------------------------- #
@pytest.fixture
def paths(monkeypatch, tmp_path):
    """Fresh module, clean env, cwd inside a temp dir."""
    monkeypatch.delenv("UAVAPF_RESULTS", raising=False)
    monkeypatch.chdir(tmp_path)
    from uavapf import paths as mod

    return mod


def _same(a: Path, b: Path) -> bool:
    return a.resolve() == b.resolve()


def test_env_var_name(paths):
    assert paths.ENV_VAR == "UAVAPF_RESULTS"


def test_default_is_cwd_results_and_not_created(paths, tmp_path):
    root = paths.results_root()
    assert root.is_absolute()
    assert _same(root, tmp_path / "results")
    assert not root.exists(), "results_root() must not create the directory"


def test_absolute_env_value(paths, monkeypatch, tmp_path):
    target = tmp_path / "hdd" / "uav_results"
    monkeypatch.setenv("UAVAPF_RESULTS", str(target))
    assert _same(paths.results_root(), target)


def test_relative_env_value_resolves_against_cwd(paths, monkeypatch, tmp_path):
    monkeypatch.setenv("UAVAPF_RESULTS", "out/runs")
    root = paths.results_root()
    assert root.is_absolute()
    assert _same(root, tmp_path / "out" / "runs")


def test_tilde_is_expanded(paths, monkeypatch, tmp_path):
    home = tmp_path / "home"
    home.mkdir()
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.setenv("USERPROFILE", str(home))  # Windows
    monkeypatch.setenv("UAVAPF_RESULTS", "~/uav")
    assert _same(paths.results_root(), home / "uav")


@pytest.mark.parametrize("blank", ["", "   "])
def test_blank_env_falls_back_to_default(paths, monkeypatch, tmp_path, blank):
    monkeypatch.setenv("UAVAPF_RESULTS", blank)
    assert _same(paths.results_root(), tmp_path / "results")


def test_env_read_at_call_time(paths, monkeypatch, tmp_path):
    first = paths.results_root()
    monkeypatch.setenv("UAVAPF_RESULTS", str(tmp_path / "later"))
    second = paths.results_root()
    assert not _same(first, second)
    assert _same(second, tmp_path / "later")


def test_ensure_dir_creates_nested_and_is_idempotent(paths, tmp_path):
    target = tmp_path / "a" / "b" / "c"
    out = paths.ensure_dir(target)
    assert out == target
    assert target.is_dir()
    assert paths.ensure_dir(target) == target  # second call must not raise
