from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CPP = ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp"
TEST = ROOT / "tests/test_product_view_reopen_lifecycle.py"

source = CPP.read_text(encoding="utf-8")


def replace_once(old: str, new: str, label: str) -> None:
    global source
    count = source.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly one match, found {count}")
    source = source.replace(old, new, 1)


replace_once(
    "#include <QTimer>\n#include <QTcpServer>",
    "#include <QTimer>\n#include <QPointer>\n#include <QTcpServer>",
    "QPointer include",
)

replace_once(
    '''      if (!embedded_web_server_is_owned_) {
        // Never reuse or terminate an untrusted endpoint. A marker mismatch
        // receives an alternate port; a refused endpoint may be claimed.
        select_owned_embedded_web_server(identity,
          !embedded_web_server_probe_.failure_detail.contains(QStringLiteral("marker does not match")));
        return;
      }''',
    '''      if (!embedded_web_server_is_owned_) {
        // A listener that failed the complete repository/resource probe is not
        // ours and must never be claimed by racing a bind on the same fixed
        // port. This also covers a stale Product View server left behind when
        // a previous Studio process was terminated abnormally. Start our owned
        // server on a freshly selected loopback port instead.
        select_owned_embedded_web_server(identity, false);
        return;
      }''',
    "failed unowned endpoint fallback",
)

replace_once(
    '''    fail_embedded_web_server_probe(identity, port, navigation_token, QStringLiteral("local server exited with code %1: %2").arg(exit_code).arg(QString::fromUtf8(process->readAll()).trimmed().left(240)));''',
    '''    const QString server_output = QString::fromUtf8(process->readAll()).trimmed();
    fail_embedded_web_server_probe(identity, port, navigation_token,
      QStringLiteral("local server exited with code %1: %2").arg(exit_code).arg(server_output.right(1200)));''',
    "server exit diagnostic tail",
)

# QWebEngine JavaScript completion callbacks are not QObject signal connections.
# Guard every long-lived ScenePreviewWidget callback that can complete after the
# widget has begun closing. A raw `this` remains convenient inside each lambda,
# but it is never dereferenced unless the QPointer proves the QObject is alive.
replace_once(
    '''  const quint64 teardown_navigation_token = embedded_web_navigation_token_;
  embedded_web_view_->page()->runJavaScript(QStringLiteral(''',
    '''  const quint64 teardown_navigation_token = embedded_web_navigation_token_;
  const QPointer<ScenePreviewWidget> lifetime_guard(this);
  embedded_web_view_->page()->runJavaScript(QStringLiteral(''',
    "loading-document guard declaration",
)
replace_once(
    '''    [this, html, scene_id, teardown_navigation_token](const QVariant & result) {
      if (!embedded_web_view_ || teardown_navigation_token != embedded_web_navigation_token_) return;''',
    '''    [this, lifetime_guard, html, scene_id, teardown_navigation_token](const QVariant & result) {
      if (!lifetime_guard) return;
      if (!embedded_web_view_ || teardown_navigation_token != embedded_web_navigation_token_) return;''',
    "loading-document guard callback",
)

replace_once(
    '''  embedded_web_view_->page()->runJavaScript(QString::fromUtf8(kStatusScript), [this, identity, navigation_token, readiness_token, expected_json_path, viewer_url](const QVariant & value) {
    if (!embedded_web_identity_is_current(identity) || navigation_token != embedded_web_navigation_token_ ||''',
    '''  const QPointer<ScenePreviewWidget> lifetime_guard(this);
  embedded_web_view_->page()->runJavaScript(QString::fromUtf8(kStatusScript), [this, lifetime_guard, identity, navigation_token, readiness_token, expected_json_path, viewer_url](const QVariant & value) {
    if (!lifetime_guard) return;
    if (!embedded_web_identity_is_current(identity) || navigation_token != embedded_web_navigation_token_ ||''',
    "readiness callback guard",
)

replace_once(
    '''    embedded_web_view_->page()->runJavaScript(QStringLiteral(
      "window.__WORKCELL_VIEWER_LIFECYCLE__?.disposeScene?.('qt_scene_navigation') || "''',
    '''    const QPointer<ScenePreviewWidget> lifetime_guard(this);
    embedded_web_view_->page()->runJavaScript(QStringLiteral(
      "window.__WORKCELL_VIEWER_LIFECYCLE__?.disposeScene?.('qt_scene_navigation') || "''',
    "scene-navigation guard declaration",
)
replace_once(
    '''      [this, identity, queued_navigation_token, viewer_url](const QVariant & result) {
        if (!embedded_web_view_ || !embedded_web_identity_is_current(identity) ||''',
    '''      [this, lifetime_guard, identity, queued_navigation_token, viewer_url](const QVariant & result) {
        if (!lifetime_guard) return;
        if (!embedded_web_view_ || !embedded_web_identity_is_current(identity) ||''',
    "scene-navigation guard callback",
)

replace_once(
    '''  embedded_web_view_->page()->runJavaScript(QString::fromUtf8(kContractProbe),
    [this, identity, navigation_token, browser_load_token, readiness_token, expected_url, attempt](const QVariant & value) {
      if (!embedded_web_view_ || !embedded_web_identity_is_current(identity) ||''',
    '''  const QPointer<ScenePreviewWidget> lifetime_guard(this);
  embedded_web_view_->page()->runJavaScript(QString::fromUtf8(kContractProbe),
    [this, lifetime_guard, identity, navigation_token, browser_load_token, readiness_token, expected_url, attempt](const QVariant & value) {
      if (!lifetime_guard) return;
      if (!embedded_web_view_ || !embedded_web_identity_is_current(identity) ||''',
    "editor-contract callback guard",
)

replace_once(
    '''  embedded_web_view_->page()->runJavaScript(script, [this, identity, state_request_token](const QVariant & value){
    if (!embedded_web_identity_is_current(identity) ||''',
    '''  const QPointer<ScenePreviewWidget> lifetime_guard(this);
  embedded_web_view_->page()->runJavaScript(script, [this, lifetime_guard, identity, state_request_token](const QVariant & value){
    if (!lifetime_guard) return;
    if (!embedded_web_identity_is_current(identity) ||''',
    "editor-command callback guard",
)

replace_once(
    '''  embedded_web_view_->page()->runJavaScript(QString::fromUtf8(kPoll), [this, identity, state_request_token](const QVariant & value){
    if (!embedded_web_identity_is_current(identity)) return;''',
    '''  const QPointer<ScenePreviewWidget> lifetime_guard(this);
  embedded_web_view_->page()->runJavaScript(QString::fromUtf8(kPoll), [this, lifetime_guard, identity, state_request_token](const QVariant & value){
    if (!lifetime_guard) return;
    if (!embedded_web_identity_is_current(identity)) return;''',
    "editor-event callback guard",
)

CPP.write_text(source, encoding="utf-8")

TEST.write_text(
    '''from pathlib import Path\n\nROOT = Path(__file__).resolve().parents[1]\nCPP = (ROOT / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text(encoding="utf-8")\n\n\ndef _block(start: str, end: str) -> str:\n    a = CPP.index(start)\n    b = CPP.index(end, a)\n    return CPP[a:b]\n\n\ndef test_failed_unowned_product_view_endpoint_never_rebinds_same_fixed_port():\n    block = _block(\n        "void ScenePreviewWidget::run_embedded_web_server_probes",\n        "void ScenePreviewWidget::fail_embedded_web_server_probe",\n    )\n    assert "select_owned_embedded_web_server(identity, false);" in block\n    assert "failure_detail.contains(QStringLiteral(\\\"marker does not match\\\"))" not in block\n\n\ndef test_owned_server_failure_keeps_exception_tail_for_actionable_diagnostics():\n    block = _block(\n        "void ScenePreviewWidget::start_owned_embedded_web_server",\n        "void ScenePreviewWidget::retire_embedded_web_navigation_for_handoff",\n    )\n    assert "server_output.right(1200)" in block\n    assert "trimmed().left(240)" not in block\n\n\ndef test_qwebengine_async_callbacks_are_lifetime_guarded():\n    assert "#include <QPointer>" in CPP\n    # These callbacks remain outstanding across scene handoff and application close.\n    for token in [\n        "[this, lifetime_guard, html, scene_id, teardown_navigation_token]",\n        "[this, lifetime_guard, identity, navigation_token, readiness_token, expected_json_path, viewer_url]",\n        "[this, lifetime_guard, identity, queued_navigation_token, viewer_url]",\n        "[this, lifetime_guard, identity, navigation_token, browser_load_token, readiness_token, expected_url, attempt]",\n        "[this, lifetime_guard, identity, state_request_token]",\n    ]:\n        assert token in CPP\n    assert CPP.count("if (!lifetime_guard) return;") >= 6\n''',
    encoding="utf-8",
)

print("R1 Product View reopen patch applied")
