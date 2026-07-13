from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
CANONICAL = REPO / "dependencies" / "emd_epd_ws.repos"
LEGACY = REPO / "tesseract.repos"
ROSDEP = REPO / "scripts" / "rosdep_overrides.yaml"
PREFLIGHT = REPO / "scripts" / "preflight_trajopt_osqp_compatibility.sh"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_canonical_manifests_pin_trajopt_033_with_osqp_v1_stack():
    for path in (CANONICAL, LEGACY):
        text = _text(path)
        assert "https://github.com/tesseract-robotics/trajopt.git" in text
        assert "version: 884e6177c4a220fc20a0c43100a0473c180a3bec" in text or "version: 0.33.0" in text
        assert "https://github.com/osqp/osqp.git" in text
        assert "version: v1.0.0" in text
        assert "https://github.com/robotology/osqp-eigen.git" in text
        assert "version: v0.11.0" in text


def test_rosdep_overrides_do_not_select_generic_jammy_libosqp():
    text = _text(ROSDEP)
    assert "osqp:\n  ubuntu: [libosqp-dev]" not in text
    assert "osqp_vendor:\n  ubuntu: [libosqp-dev]" not in text
    assert "trajopt 0.33.0 requires OSQP v1 API" in text
    assert "ubuntu: []" in text


def test_preflight_detects_osqp_api_major_mismatch_symbols():
    text = _text(PREFLIGHT)
    for new_symbol in [
        "OSQPSolver",
        "OSQPCscMatrix",
        "OSQPInt",
        "osqp_update_data_vec",
        "osqp_update_data_mat",
        "warm_starting",
        "OSQP_NO_ERROR",
        "OSQP_ALGEBRA_LOAD_ERROR",
    ]:
        assert new_symbol in text
    for old_symbol in ["OSQPWorkspace", "warm_start"]:
        assert old_symbol in text


def test_preflight_reports_stale_duplicate_and_wrong_provider_cases():
    text = _text(PREFLIGHT)
    assert "multiple trajopt_sco packages found" in text
    assert "not inside a git checkout" in text
    assert "wrong TrajOpt commit" in text
    assert "wrong OSQP source" in text
    assert "OSQP v1-compatible headers were found from /usr" in text
    assert "Corrective command" in text


def test_helper_scripts_skip_rosdep_osqp_and_run_preflight():
    fix = _text(REPO / "fix_and_build_humble.sh")
    deps = _text(REPO / "scripts" / "lib" / "dependencies.sh")
    assert "preflight_trajopt_osqp_compatibility.sh" in fix
    for key in ["osqp", "osqp_vendor", "osqp-eigen", "osqp_eigen"]:
        assert key in fix
        assert key in deps
