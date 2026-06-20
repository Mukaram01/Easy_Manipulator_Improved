# Workcell Cleanup Audit

## Purpose

The Workcell cleanup audit exists to produce a factual cleanup report before deleting anything from the repository or generated workspace outputs. It is an evidence-gathering step only: reviewers should use the report to understand candidate files, confidence levels, and safety flags before deciding whether a separate deletion PR is warranted.

## Command

Run the audit from the repository root:

```bash
python3 scripts/audit_workcell_cleanup_candidates.py
```

## Outputs

The audit writes both machine-readable and reviewer-friendly reports under `build/workcell_cleanup_audit/`:

- `build/workcell_cleanup_audit/cleanup_candidates.json`
- `build/workcell_cleanup_audit/cleanup_candidates.md`

## Confidence interpretation

- `high`: likely generated junk such as caches, logs, or backups.
- `medium`: generated and likely reproducible, but provenance should be reviewed first.
- `low`: static-analysis guess requiring human review.

## Safety interpretation

Only candidates marked `safe_to_delete_now: true` should be considered for a later deletion PR.

Candidates marked `dangerous_do_not_delete` and any candidate with `safe_to_delete_now: false` must not be removed without a specific follow-up investigation. Treat these entries as blockers for automatic cleanup, not as deletion recommendations.

## Explicit non-goals

This audit does not:

- delete files in this PR;
- change scene generation behavior;
- change Workcell Builder UI behavior;
- change EPD ownership boundaries;
- weaken fake-hardware-first safety gates.
