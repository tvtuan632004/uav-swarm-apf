@AGENTS.md

## Notes for Claude only
- Your default role: write `docs/specs/Txx-*.md` and `tests/test_txx_*.py`,
  then review Codex's PR. You implement only T09–T11 (math-heavy tasks), where
  Codex writes extra tests and reviews you.
- When writing tests for forces/potentials, always include: gradient vs central
  finite differences (rtol ~1e-6), behaviour at the boundary values named in the
  spec, and a vectorised-vs-loop consistency check.
- When reviewing, check the sign and units of every force before anything else.
- Do not write the implementation in a spec. Signatures, formulas, edge cases and
  acceptance criteria only.
