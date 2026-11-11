# AGENTS.md — uav-swarm-apf

Shared rules for every AI agent (Codex, Claude Code) working in this repo.
Read this file, then `docs/handoff.md` (newest entry first), then the spec of
your current task in `docs/specs/`, before writing anything. The task table,
roles and schedule are in `docs/plan.md`.

## 1. Project in three lines
Formation control and collision avoidance for a UAV swarm (double-integrator
model) using a modified artificial potential field: goal-normalized repulsion
(T1), Morse + barrier inter-agent potential (T2), gyroscopic force with stuck
reset (T3), PSD-projected Hessian damping (T4). Compared against baselines
B1–B5 on scenarios S1–S5. **`paper/main.tex` is the single source of truth for
every formula.** If code and paper disagree, stop and report it in the handoff.

## 2. Standard commands (run from repo root)
```
uv sync                      # install / update the environment
uv run pytest -q             # all tests
uv run ruff check .          # lint
uv run ruff format --check . # formatting
```
Python 3.12, dependencies managed only through `uv` (`uv add <pkg>`, never `pip install`).

## 3. Code conventions
- SI units everywhere (m, s, m/s, m/s²). Write the unit in the docstring of every
  physical input/output.
- `float64` for all numerical arrays. Positions/velocities have shape `(N, 3)`.
- Force/potential functions are **pure**: no hidden state, no globals, no I/O.
- All randomness goes through a `numpy.random.Generator` passed in explicitly
  (created from a seed). Never use `np.random.<func>` module-level calls.
- Paths via `pathlib.Path`, never string concatenation. Code must run on both
  Windows and Ubuntu.
- Every formula in code cites its source in a comment: `# eq. (12), main.tex`
  or `# Olfati-Saber 2006, p. 405, eq. (21)`. Baseline parameters must be
  checked against the original paper, not recalled from memory.
- Type hints on public functions; NumPy-style docstrings.
- Results never go into the repo. Output root comes from env var
  `UAVAPF_RESULTS` (see `src/uavapf/paths.py`), default `./results/`.

## 4. Workflow
1. One task = one small, reviewable change (target < ~400 changed lines), normally
   committed by the human. The GitHub remote is recorded in D011. Commit messages
   start with the task code.
2. Roles per task are in the task table (`docs/plan.md`, section 3). Default:
   Claude writes spec + tests, Codex implements, Claude reviews. Codex writes a
   spec only when the user asks for it explicitly.
3. **The implementer never edits `tests/`** unless the spec explicitly allows
   it. If a test looks wrong, stop and explain why in `docs/handoff.md`.
4. Tests and lint must be green before asking for review. Review is done
   against the spec, not against taste.
5. Never commit, push or rewrite git history yourself unless the human explicitly
   requests that action for the current turn. Otherwise only the human commits,
   after `scripts\setup.cmd` reports ALL DONE.
6. Never run two agents in the same working directory at the same time. For
   parallel work use separate `git worktree`s.
7. No tokens, keys or passwords in the repo or in prompts.
8. Design decisions that outlive one task go into `docs/decisions.md`
   (3–5 lines each, dated).
9. Take turns: only one agent works in the folder at a time. Start a turn with
   `git status` (must be clean or only contain the previous agent's handoff),
   end it with a handoff entry; the human commits between turns.

## 5. Codex on Windows: isolated environment (D007)
Codex's sandbox must not touch the user's `.venv` or `%TEMP%`. Before any `uv` or
`pytest` command, set:
```
$env:UV_PROJECT_ENVIRONMENT = ".venv-codex"
$env:PYTEST_DEBUG_TEMPROOT  = ".pytest_tmp/codex"
```
Claude works on a cloud copy and syncs files; the user verifies on Windows with
`scripts\setup.cmd`. Both `.venv-codex/` and `.pytest_tmp/` are git-ignored.

## 6. Ending a session
Before you stop, add an entry at the **top** of `docs/handoff.md`:

```
## YYYY-MM-DD · Txx · <Agent> → <Agent>
- Done: ...
- Tests: x/y green (command used)
- Open issues: ...
- Next: ...
```

## 7. Review output format
Findings are listed as **blocking / should fix / nit**, each with `file:line`
and a one-line reason. End with a verdict: `approve` or `changes requested`.
