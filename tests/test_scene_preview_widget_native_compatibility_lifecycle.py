"""Regression contract for the retained native Product View compatibility viewport."""
from pathlib import Path


CPP = (Path(__file__).resolve().parents[1] / "workcell_builder/workcell_builder/gui/scene_preview_widget.cpp").read_text()
HDR = (Path(__file__).resolve().parents[1] / "workcell_builder/workcell_builder/gui/scene_preview_widget.h").read_text()


def test_repeated_web3d_fallback_reuses_one_owned_viewport_and_latest_payload():
    assert "Scene3DViewportWidget * compatibility_scene3d_viewport_{ nullptr };" in HDR
    fallback = CPP[CPP.index("void ScenePreviewWidget::activate_native_compatibility_preview"):CPP.index("QString ScenePreviewWidget::runtime_preview_status_text")]

    # Allocation is guarded, while ingestion and selection restoration happen on
    # every activation/retry so the retained viewport cannot show stale content.
    assert "if (!compatibility_scene3d_viewport_)" in fallback
    assert "compatibility_scene3d_viewport_ = new Scene3DViewportWidget(view3d_container_);" in fallback
    assert "compatibility_scene3d_viewport_->ingest_preview_items(preview_items_);" in fallback
    assert "compatibility_scene3d_viewport_->selected_id = selected_preview_item_id_;" in fallback
    assert "compatibility_scene3d_viewport_->fit_product_view();" in fallback
    assert "if (embedded_web_view_) embedded_web_view_->setVisible(false);" in fallback
    assert "simple_3d_view_ = native;" not in fallback

    updates = CPP[CPP.index("void ScenePreviewWidget::set_preview_items"):CPP.index("void ScenePreviewWidget::set_preview_scene_name")]
    assert "auto * viewport = active_native_viewport();" in updates
    assert "if (viewport) viewport->ingest_preview_items(preview_items_);" in updates
    assert "is_native_product_view_backend() ?" not in updates


def test_web3d_ready_restores_its_single_view_and_retains_compatibility_instance():
    ready = CPP[CPP.index('if (boot_state == QStringLiteral("ready")'):CPP.index("set_embedded_product_view_state(EmbeddedProductViewState::Loading", CPP.index('if (boot_state == QStringLiteral("ready")'))]
    restore = CPP[CPP.index("void ScenePreviewWidget::show_embedded_web_product_view"):CPP.index("void ScenePreviewWidget::set_preview_context")]

    assert "show_embedded_web_product_view();" in ready
    assert "compatibility_scene3d_viewport_->setVisible(false);" in restore
    assert "embedded_web_view_->setVisible(true);" in restore
    assert "delete compatibility_scene3d_viewport_" not in CPP
