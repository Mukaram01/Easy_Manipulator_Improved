# Workcell Studio Template Instantiation

New Cell now calls a production template-instantiation backend (`instantiate_workcell_studio_template`) that creates a complete, discoverable scene package.

Generated artifacts include:
- `package.xml`, `CMakeLists.txt`
- `environment.yaml` (with default gripper mount RPY `-1.5708 -1.5708 0`)
- `scene_manifest.yaml`
- `config/task_recipe.yaml`
- `config/workcell_builder_task_intent.yaml`
- `preview/static_preview.html`
- `smoke/offline_smoke_report.json|.html|summary.txt`

Safety behavior remains unchanged:
- fake hardware first
- runtime execution disabled
- no robot motion commanded
- launch hint uses `use_fake_hardware:=true`

Name collisions are handled safely via automatic suffixing (`scene_2`, `scene_3`, ...).
Placeholder robot selections are marked `PREVIEW_ONLY`.
