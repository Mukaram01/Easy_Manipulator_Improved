from pathlib import Path


HEADER = Path("workcell_builder/workcell_builder/gui/scene_preview_widget.h").read_text(encoding="utf-8")


def test_product_view_seeds_each_widget_with_a_free_loopback_port():
    """A stale server from a crashed Studio process must not poison the next launch."""
    assert "#include <QtNetwork/QTcpServer>" in HEADER
    assert "#include <QtNetwork/QHostAddress>" in HEADER
    assert "socket.listen(QHostAddress::LocalHost, 0)" in HEADER
    assert "static_cast<int>(socket.serverPort())" in HEADER
    assert "int embedded_web_server_port_{ 8765 };" not in HEADER


def test_fixed_8765_is_only_a_last_resort_if_ephemeral_selection_fails():
    member_start = HEADER.index("int embedded_web_server_port_{")
    member_end = HEADER.index("QGraphicsView * fallback_2d_view_", member_start)
    initializer = HEADER[member_start:member_end]

    assert initializer.index("socket.listen(QHostAddress::LocalHost, 0)") < initializer.index("return 8765;")
