from pathlib import Path


def test_grasp_strategy_fixtures_smoke() -> None:
    # Intentionally dependency-free CI smoke test (no PyYAML). Full schema
    # validation can be added later when scripts/validate_grasp_strategy.py exists.
    fixture_dir = Path(__file__).resolve().parents[1] / "fixtures" / "grasp_strategies"
    assert fixture_dir.is_dir(), "Missing tests/fixtures/grasp_strategies directory"

    yaml_files = sorted(fixture_dir.glob("*.yaml"))
    assert yaml_files, "Expected at least one .yaml grasp strategy fixture"

    fixture_names = [p.name.lower() for p in yaml_files]
    assert any("suction" in name for name in fixture_names), "Expected a suction-related fixture"
    assert any("finger" in name or "parallel" in name for name in fixture_names), (
        "Expected a finger/parallel-gripper-related fixture"
    )

    for fixture in yaml_files:
        text = fixture.read_text(encoding="utf-8")
        assert text.strip(), f"Fixture is empty: {fixture.name}"

        non_comment_lines = [
            line for line in text.splitlines() if line.strip() and not line.lstrip().startswith("#")
        ]
        assert any(":" in line for line in non_comment_lines), (
            f"Fixture does not look YAML-like (no key:value line): {fixture.name}"
        )
