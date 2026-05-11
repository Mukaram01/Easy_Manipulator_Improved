# Workcell Asset/Profile Catalog

Defines metadata-only catalogs for Workcell Studio: robot profiles, tool profiles, pair compatibility, camera profiles, and environment asset hints.

## Validate
- `python3 scripts/validate_workcell_asset_catalog.py --repo-root .`
- `python3 scripts/validate_workcell_asset_catalog.py --repo-root . --strict`

Markers:
- `WORKCELL_ASSET_CATALOG: PASS`
- `WORKCELL_ASSET_CATALOG: WARN`
- `WORKCELL_ASSET_CATALOG: FAIL`

Warnings indicate incomplete hints (non-blocking). Failures indicate missing required fields, duplicates, or invalid status/type values.

## Safe profile extension
1. Add profile JSON under appropriate folder.
2. Keep IDs lowercase snake_case.
3. Include required TCP/mount/planning/reach/topic metadata.
4. Run validator in strict mode and fix all warnings.

## Fake-hardware-first
Catalogs are offline metadata/export hints only. No runtime motion, no MoveIt planning calls, and no real hardware enablement.
