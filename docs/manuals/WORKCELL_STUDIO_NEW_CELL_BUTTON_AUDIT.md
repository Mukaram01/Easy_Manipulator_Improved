# Workcell Studio New Cell Button Audit (Point 1: UI Button Audit)

This audit covers the exact workflow: **Workspace → New Cell from Scratch → Generate → Validate → Plan & Simulate**.

| Workflow action | Visible label | Page/location | Handler / slot / function | Expected result | Next recommended action | Tier | Type | Safety mode |
|---|---|---|---|---|---|---|---|---|
| Select/confirm workspace | Choose Workspace / workspace selector | New Scene dialog (`scene_select`) | `SceneSelect::on_choose_workspace_button_clicked` | Workspace root selected and displayed | New Cell / New Scene | Primary | Navigation + config | design |
| New Cell entry | New Cell | Command Bar / Dashboard | Command bar lambda (`label == "New Cell"`) | Opens Scene Builder new-cell flow | New Scene | Primary | Navigation | design |
| New Scene dialog | New Scene | Scene creation dialog | `SceneSelect::on_new_scene_button_clicked` | Starts new scene metadata flow | Set cell/scene name | Primary | Execution | design |
| Apply defaults | Use Recommended Layout | Scene Builder | `MainWindow::use_recommended_layout` | UR5 + Robotiq 2F + table/pick/place/camera defaults staged | Add to Canvas | Primary | Execution | design |
| Add asset | Add to Canvas | Asset Catalog card | lambda -> `MainWindow::add_asset_to_canvas_from_catalog` | Selected asset instance added to canvas | Save Layout | Primary | Execution | design |
| Persist layout | Save Layout | Scene Builder toolbar | `MainWindow::save_layout_changes` | `environment_layout.yaml` written | Generate/Update Task Intent | Primary | Execution | design |
| Remove layout instance | Remove Selected Layout Item | Scene Builder More Actions | `MainWindow::delete_selected_item` | Selected canvas item removed (guarded for robot base) | Save Layout | More Actions | Execution | design |
| Layout-only check | Validate Layout | Validation page | `MainWindow::run_layout_validation_only` | Runs static layout checks and blockers/warnings update | Generate/Update Task Intent or Run Offline Validation | Secondary | Execution | design |
| Task intent generation | Generate/Update Task Intent | Scene Builder Task Intent card | `MainWindow::generate_or_update_task_intent_for_selected_scene` | Task intent file generated/updated | Generate Scene Package | Primary | Execution | plan |
| Open task file | Open Task File | Scene Builder More Actions | `MainWindow::open_selected_task_file` | Opens generated task intent YAML | Copy Task Summary / Generate Scene Package | More Actions | Navigation | plan |
| Task summary clipboard | Copy Task Summary | Scene Builder More Actions | `MainWindow::copy_selected_task_summary` | Copies concise task summary text | Generate Scene Package | More Actions | Execution | plan |
| Scene package generation | Generate Scene Package | Command Bar + Scene Builder | Command bar lambda (`label == "Generate Scene Package"`) -> `MainWindow::run_layout_merge_for_selected_scene(true)` | Package artifacts regenerated | Refresh Existing Scenes | Primary | Execution | plan |
| Refresh scene list | Refresh Existing Scenes | Scene browser (`scene_select`) | `SceneSelect::on_refresh_scenes_button_clicked` | Existing scenes table refreshed | Run Offline Validation | Secondary | Execution | design |
| Offline validation | Run Offline Validation | Validation page | `MainWindow::run_offline_validation` | Offline acceptance/smoke validations executed | Generate Readiness Pack | Primary | Execution | plan |
| Readiness pack | Generate Readiness Pack | Validation page | `MainWindow::generate_readiness_pack` | Readiness JSON/HTML artifacts refreshed | Open Readiness Dashboard | Primary | Execution | plan |
| Readiness dashboard | Open Readiness Dashboard | Validation More Actions | `MainWindow::open_readiness_dashboard` | Opens dashboard artifact | Open Plan & Simulate | More Actions | Navigation | plan |
| Enter run console | Open Plan & Simulate | Dashboard / Demo navigation | `dash_preview` lambda / `go_preview` lambda (`studio_nav_->setCurrentRow(7)`) | Opens Plan & Simulate page and command panel | Open RViz2 / MoveIt | Secondary | Navigation | plan |
| Build + RViz2 | Open RViz2 / MoveIt | Plan & Simulate page | `MainWindow::run_preview_build` | Executes build command and prepares RViz2/MoveIt flow | Run Fake-Hardware Simulation | Secondary | Execution | plan |
| Fake-hardware run | Run Fake-Hardware Simulation | Plan & Simulate page | `MainWindow::run_fake_hardware_preview` | Launches fake-hardware simulation command only | Stop Simulation / Copy Launch Command | Primary | Execution | simulate fake hardware |
| Stop running process | Stop Simulation | Plan & Simulate page | `MainWindow::stop_preview_process` | Stops running preview process and logs completion | Copy Launch Command or rerun | Secondary | Execution | simulate fake hardware |
| Launch clipboard | Copy Launch Command | Plan & Simulate + Existing Scenes table | lambda copying `selected_scene_launch_command()` | Copies launch command (`use_fake_hardware:=true`) | Open RViz2 / MoveIt or Run Fake-Hardware Simulation | Secondary | Execution | simulate fake hardware |

## New Cell Action Map

**Workspace → New Cell → Layout → Task Intent → Generate Scene Package → Validate → Plan & Simulate**

This map is also surfaced in UI logs as a compact helper line.
