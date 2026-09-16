#include "task_intent_model.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

TEST(TaskIntentModel, CanonicalHashGoldenIsStableAcrossYamlFormatting)
{
  const std::string a = "b: 2\na: [0.0, 1]\n";
  const std::string b = "# comment\na: [0, 1.000000]\nb: 2.0\n";
  EXPECT_EQ(workcell_builder::canonical_task_intent_sha256_for_testing(a), workcell_builder::canonical_task_intent_sha256_for_testing(b));
  EXPECT_EQ(workcell_builder::canonical_task_intent_sha256_for_testing(a), "6b5a43d11c069f69bf75b5de430b4caf6a36fb7a8e7e5840c15130ea44f92fd3");
}

TEST(TaskIntentModel, CanonicalRepresentationPreservesScalarTypesAndArrayOrder)
{
  const auto numeric = workcell_builder::canonical_task_intent_json_for_testing("x: 1\ny: true\nz: null\na: [1, 2]\n");
  const auto strings = workcell_builder::canonical_task_intent_json_for_testing("x: '1'\ny: 'true'\nz: 'null'\na: [2, 1]\n");
  EXPECT_NE(numeric, strings);
  EXPECT_NE(workcell_builder::canonical_task_intent_sha256_for_testing("x: 1\n"), workcell_builder::canonical_task_intent_sha256_for_testing("x: '1'\n"));
  EXPECT_NE(workcell_builder::canonical_task_intent_sha256_for_testing("x: true\n"), workcell_builder::canonical_task_intent_sha256_for_testing("x: 'true'\n"));
  EXPECT_NE(workcell_builder::canonical_task_intent_sha256_for_testing("x: null\n"), workcell_builder::canonical_task_intent_sha256_for_testing("x: 'null'\n"));
}

TEST(TaskIntentModel, ParsesTheTypedV2AuthoredShape)
{
  const auto model = workcell_builder::TaskIntentModel::from_yaml(
    "schema: workcell_builder_task_intent/v2\n"
    "task: {id: t, type: pick_place, template: pick_place}\n"
    "pick: {selection: {source_ref: detected_objects/v1, source_type: perception, zone_ref: pick_zone_main, object_filter: {class_id: bottle, color: red, min_confidence: 0.8, max_age_seconds: 2.0}}, grasp: {policy: EXACT, required_capability: two_finger_parallel, strategy_ref: top_2f, approach: {axis: z_down, distance_m: 0.12}, orientation: {mode: vertical, allowed_roll_deg: [0, 90], allowed_yaw_deg: [0, 180], tolerance_rad: [0.1, 0.2, 0.3]}, tcp_offset_xyz_m: [1, 2, 3], tcp_offset_rpy_rad: [0.01, 0.02, 0.03], contact: {required: true, min_quality: 0.4}, aperture: {min_m: 0.01, max_m: 0.08}, lift: {axis: x_minus, distance_m: 0.15}}}\n"
    "place: {target: {asset_ref: target_bin_default, region_ref: default_drop_zone}, placement: {policy: EXACT, requested_local_pose: {xyz_m: [0.01, 0.02, 0.03], rpy_rad: [0.1, 0.2, 0.3]}, orientation: {mode: fixed, rpy_rad: [0.2, 0.3, 0.4], tolerance_rad: [0.01, 0.02, 0.03]}, approach: {axis: z_down, distance_m: 0.1}, clearance_m: 0.05, retreat: {axis: y_plus, distance_m: 0.1}}, release: {strategy: tool_release}}\n"
    "routing: {mode: direct}\nscene_package: scenes/ur5_2f_test\nsafety: {execution_mode: simulation_preview, require_fake_hardware: true, real_hardware_enabled: false, preview_policy: diagnostic_if_unresolved}\n");
  EXPECT_EQ(model.task_id, "t");
  EXPECT_EQ(model.pick_selection.zone_ref, "pick_zone_main");
  EXPECT_EQ(model.grasp.strategy_ref, "top_2f");
  EXPECT_EQ(model.place.asset_ref, "target_bin_default");
  EXPECT_EQ(model.release_strategy, "tool_release");
  EXPECT_EQ(model.scene_package, "scenes/ur5_2f_test"); EXPECT_FALSE(model.routing_yaml.empty());
  ASSERT_TRUE(model.pick_selection.color.has_value()); EXPECT_EQ(*model.pick_selection.color, "red"); ASSERT_TRUE(model.pick_selection.min_confidence.has_value()); EXPECT_DOUBLE_EQ(*model.pick_selection.min_confidence, 0.8); ASSERT_TRUE(model.pick_selection.max_age_seconds.has_value()); EXPECT_DOUBLE_EQ(*model.pick_selection.max_age_seconds, 2.0);
  EXPECT_EQ(model.grasp.approach_axis, "z_down"); EXPECT_DOUBLE_EQ(model.grasp.approach_distance_m, 0.12); EXPECT_EQ(model.grasp.orientation_mode, "vertical");
  EXPECT_DOUBLE_EQ(model.grasp.tcp_offset_xyz_m[2], 3.0); EXPECT_DOUBLE_EQ(model.grasp.contact_min_quality, 0.4); EXPECT_DOUBLE_EQ(model.grasp.aperture_max_m, 0.08); EXPECT_EQ(model.grasp.lift_axis, "x_minus");
  ASSERT_TRUE(model.place.requested_local_pose.has_value()); EXPECT_DOUBLE_EQ(model.place.requested_local_pose->rpy_rad[2], 0.3); EXPECT_EQ(model.place.orientation_mode, "fixed"); EXPECT_EQ(model.place.approach_axis, "z_down"); EXPECT_EQ(model.place.retreat_axis, "y_plus"); EXPECT_DOUBLE_EQ(model.place.clearance_m, 0.05);
}

TEST(TaskIntentModel, NullableAndVariableLengthValuesRemainSemantic)
{
  const auto model = workcell_builder::TaskIntentModel::from_yaml(
    "schema: workcell_builder_task_intent/v2\ntask: {id: t, type: pick_place, template: pick_place}\n"
    "pick: {selection: {source_ref: s, source_type: perception, zone_ref: z, object_filter: {class_id: c, color: null, min_confidence: null}}, grasp: {policy: AUTO, strategy_ref: null, orientation: {allowed_roll_deg: [0,90,180,270,360], allowed_yaw_deg: [0]}}}\n"
    "place: {target: {asset_ref: a, region_ref: r}, placement: {policy: AUTO, requested_local_pose: null}}\nsafety: {}\n");
  EXPECT_FALSE(model.pick_selection.color.has_value()); EXPECT_FALSE(model.pick_selection.min_confidence.has_value()); EXPECT_FALSE(model.grasp.strategy_ref.has_value());
  EXPECT_EQ(model.grasp.allowed_roll_deg.size(), 5u); EXPECT_EQ(model.grasp.allowed_yaw_deg.size(), 1u); EXPECT_FALSE(model.place.requested_local_pose.has_value());
}

TEST(TaskIntentModel, AuthoritativeHashRequiresValidatedModel)
{
  const std::string yaml = "schema: workcell_builder_task_intent/v2\ntask: {id: t, type: pick_place}\npick: {selection: {object_filter: {class_id: bottle}}, grasp: {policy: auto, required_capability: two_finger_parallel}}\nplace: {target: {asset_ref: bin, region_ref: drop}, placement: {policy: exact, requested_local_pose: {xyz_m: [0,0,0], rpy_rad: [0,0,0]}, orientation: {}, approach: {}, retreat: {}}, release: {strategy: tool_release}}\nsafety: {execution_mode: simulation_preview}\n";
  EXPECT_FALSE(workcell_builder::TaskIntentModel::from_validated_yaml("schema: bad\n"));
  auto validated = workcell_builder::TaskIntentModel::from_validated_yaml(yaml);
  ASSERT_TRUE(validated.has_value()); EXPECT_EQ(validated->grasp.policy, "AUTO"); EXPECT_EQ(validated->place.policy, "EXACT"); EXPECT_NO_THROW(workcell_builder::authoritative_task_intent_sha256(*validated));
  EXPECT_THROW(workcell_builder::authoritative_task_intent_sha256(workcell_builder::TaskIntentModel::from_yaml(yaml)), std::invalid_argument);
}

TEST(TaskIntentModel, InvalidV2NeverGetsAuthoritativeHash)
{
  const std::string base = "schema: workcell_builder_task_intent/v2\ntask: {id: t}\npick: {selection: {object_filter: {class_id: c}}, grasp: {policy: AUTO, required_capability: two_finger_parallel}}\nplace: {target: {asset_ref: a, region_ref: r}, placement: {policy: AUTO}, release: {strategy: tool_release}}\nsafety: {}\n";
  const std::vector<std::string> bad_cases = {
    "schema: workcell_builder_task_intent/v2\ntask: {id: t}\npick: {selection: {}}\nplace: {target: {asset_ref: a, region_ref: r}, placement: {policy: AUTO}, release: {strategy: tool_release}}\nsafety: {}\n",
    "schema: workcell_builder_task_intent/v2\ntask: {id: t}\npick: {selection: {object_filter: {class_id: c}}, grasp: {required_capability: two_finger_parallel}}\nplace: {target: {asset_ref: a, region_ref: r}, placement: {policy: AUTO}, release: {strategy: tool_release}}\nsafety: {}\n",
    "schema: workcell_builder_task_intent/v2\ntask: {id: t, target_policy: {class_id: c}}\npick: {selection: {object_filter: {class_id: c}}, grasp: {policy: AUTO, required_capability: two_finger_parallel}}\nplace: {target: {asset_ref: a, region_ref: r}, placement: {policy: AUTO}, release: {strategy: tool_release}}\nsafety: {}\n",
    "schema: workcell_builder_task_intent/v2\ntask: {id: t}\npick: {selection: {object_filter: {class_id: c}}, grasp: {policy: EXACT, required_capability: two_finger_parallel}}\nplace: {target: {asset_ref: a, region_ref: r}, placement: {policy: AUTO}, release: {strategy: tool_release}}\nsafety: {}\n",
    "schema: workcell_builder_task_intent/v2\ntask: {id: t}\npick: {selection: {object_filter: {class_id: c}}, grasp: {policy: AUTO, required_capability: two_finger_parallel}}\nplace: {target: {asset_ref: a, region_ref: r}, placement: {policy: AUTO}, release: {strategy: tool_release}}\ntool: installed\nsafety: {}\n",
    "schema: workcell_builder_task_intent/v2\ntask: {id: t}\npick: {selection: {object_filter: {class_id: c}}, grasp: {policy: AUTO, required_capability: two_finger_parallel}}\nplace: {target: {asset_ref: a, region_ref: r}, placement: {policy: AUTO}, release: {strategy: tool_release}}\nsafety: {motion_started: true}\n",
  };
  for (std::size_t i = 0; i < bad_cases.size(); ++i)
    EXPECT_FALSE(workcell_builder::TaskIntentModel::from_validated_yaml(bad_cases[i])) << i;
  EXPECT_FALSE(workcell_builder::TaskIntentModel::from_validated_yaml("schema: workcell_builder_task_intent/v2\ntask: {}\n"));
}

TEST(TaskIntentModel, SharedGoldenFixturesMatchCanonicalBytesAndHashes)
{
  const auto fixtures = YAML::LoadFile(TASK_INTENT_GOLDEN_PATH)["fixtures"];
  ASSERT_TRUE(fixtures && fixtures.IsSequence());
  for (const auto & fixture : fixtures) {
    const auto yaml = fixture["yaml"].as<std::string>();
    EXPECT_EQ(workcell_builder::canonical_task_intent_json_for_testing(yaml), fixture["canonical"].as<std::string>());
    EXPECT_EQ(workcell_builder::canonical_task_intent_sha256_for_testing(yaml), fixture["sha256"].as<std::string>());
  }
}

TEST(TaskIntentModel, PresenceAndPoseScalarParity)
{
  const std::string valid = "schema: workcell_builder_task_intent/v2\ntask: {id: t}\npick: {selection: {object_filter: {class_id: c}}, grasp: {policy: AUTO, required_capability: two_finger_parallel}}\nplace: {target: {asset_ref: a, region_ref: r}, placement: {policy: PREFERRED, requested_local_pose: {xyz_m: [0,0,0], rpy_rad: [0,0,0]}}, release: {strategy: tool_release}}\nsafety: {}\n";
  EXPECT_TRUE(workcell_builder::TaskIntentModel::from_validated_yaml(valid).has_value());
  for (const auto & bad : {"object_filter: null", "tool: null", "motion_started: null", "runtime_io_applied: null", "ros_launch_started: null"}) {
    auto text = valid;
    if (std::string(bad).find("object_filter") == 0) text.replace(text.find("object_filter: {class_id: c}"), 29, bad);
    else if (std::string(bad).find("tool") == 0) text += std::string("tool: null\n");
    else text.replace(text.find("safety: {}"), 11, std::string("safety: {") + bad + "}");
    EXPECT_FALSE(workcell_builder::TaskIntentModel::from_validated_yaml(text));
  }
  auto quoted = valid; quoted.replace(quoted.find("[0,0,0]"), 7, "[\"0\",0,0]");
  EXPECT_FALSE(workcell_builder::TaskIntentModel::from_validated_yaml(quoted));
}

TEST(TaskIntentModel, FieldEditsPreserveUneditedValuesAndInvalidateHash)
{
  const std::string text = "schema: workcell_builder_task_intent/v2\ntask: {id: t}\npick: {selection: {source_ref: source, object_filter: {class_id: bottle, min_confidence: null}}, grasp: {policy: AUTO, required_capability: two_finger_parallel}}\nplace: {target: {asset_ref: bin, region_ref: drop}, placement: {policy: AUTO}, release: {strategy: tool_release}}\nsafety: {real_hardware_enabled: false}\nprovenance: {custom: preserved}\n";
  auto model = *workcell_builder::TaskIntentModel::from_validated_yaml(text);
  model.set_field({"pick", "selection", "object_filter", "class_id"}, "\"part\"");
  EXPECT_EQ(model.pick_selection.class_id, "part");
  EXPECT_FALSE(model.validated);
  auto saved = YAML::Load(model.to_yaml());
  EXPECT_TRUE(saved["pick"]["selection"]["object_filter"]["min_confidence"].IsNull());
  EXPECT_EQ(saved["provenance"]["custom"].as<std::string>(), "preserved");
  auto reopened = workcell_builder::TaskIntentModel::from_validated_yaml(model.to_yaml());
  ASSERT_TRUE(reopened);
  EXPECT_EQ(reopened->pick_selection.source_ref, "source");
  EXPECT_NE(workcell_builder::authoritative_task_intent_sha256(*reopened),
            workcell_builder::authoritative_task_intent_sha256(*workcell_builder::TaskIntentModel::from_validated_yaml(text)));
}
