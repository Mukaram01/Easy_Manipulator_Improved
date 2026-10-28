"""R2.5A: a fake-hardware sorting session observing simulator-rendered, EPD-classified frames of its own world."""
from __future__ import annotations

import copy
import math

import pytest

from scripts import sorting_session as ss
from scripts.runtime_pick_inputs import RENDER_PRODUCER, normalize, rendered_frame, world_revision
from tests.test_class_routing import routed_intent, three_bins
from tests.test_sorting_session import RULES, completed_summary

CAPTURE_NS = 1_000_000_000_000


def world(*objects):
    """[(object_id, x, y, z, dims)] at rest (rpy 0) as the session renders them."""
    return [{"object_id": oid, "pose": {"frame_id": "world", "xyz": [x, y, z], "rpy": [0.0, 0.0, 0.0]},
             "dimensions": list(dims)} for oid, x, y, z, dims in objects]


def frame(objects, classes, capture_ns=CAPTURE_NS, revision=None):
    """A rendered frame as perception_replay.session_frame writes it: world geometry, EPD classes, capture time."""
    items = []
    for o in objects:
        cls = classes.get(o["object_id"], ss.UNPERCEIVED)
        items.append({"object_id": o["object_id"], "class_id": cls, "confidence": None if cls == ss.UNPERCEIVED else 0.8,
                      "timestamp": capture_ns / 1e9, "pose": copy.deepcopy(o["pose"]), "dimensions": list(o["dimensions"]),
                      "perception": {"producer": RENDER_PRODUCER}})
    return {"schema_version": "detected_objects/v1",
            "source": {"mode": "simulator_rendered", "producer": RENDER_PRODUCER, "provenance": "SIMULATOR_RENDERED_SYNTHETIC",
                       "camera_source": "simulated_rgbd", "capture_wall_ns": capture_ns,
                       "world_revision": revision or world_revision(objects)}, "objects": items}


class Geometry:
    """The planner's pose helpers, enough for normalize()."""
    @staticmethod
    def quaternion_from_rpy(rpy):
        r, p, y = (v / 2 for v in rpy)
        return [math.sin(r) * math.cos(p) * math.cos(y) - math.cos(r) * math.sin(p) * math.sin(y),
                math.cos(r) * math.sin(p) * math.cos(y) + math.sin(r) * math.cos(p) * math.sin(y),
                math.cos(r) * math.cos(p) * math.sin(y) - math.sin(r) * math.sin(p) * math.cos(y),
                math.cos(r) * math.cos(p) * math.cos(y) + math.sin(r) * math.sin(p) * math.sin(y)]


def ingest(s, snapshot):
    s.ingest_frame(snapshot, normalize(snapshot, CAPTURE_NS / 1e9 + 1, Geometry), source_sha256="f" * 64)


INITIAL = world(("cup-1", 0.30, -0.1, 0.05, (0.06, 0.05, 0.10)), ("bowl-1", 0.50, -0.1, 0.05, (0.06, 0.05, 0.10)),
                ("thing-1", 0.40, -0.1, 0.05, (0.06, 0.05, 0.10)))


def session():
    intent = routed_intent(RULES)
    intent["routing"].update(unrouted_policy="stop_session", max_attempts_per_object=2)
    s = ss.SortingSession(intent, three_bins(), session_id="r", mode="continuous", clock=lambda: 100.0)
    s.initial_world = copy.deepcopy(INITIAL)
    return s


def test_a_rendered_frame_keeps_its_capture_time_and_names_the_world_it_shows():
    good = frame(INITIAL, {"cup-1": "cup"})
    assert rendered_frame(good)["world_revision"] == world_revision(INITIAL)
    assert normalize(good, CAPTURE_NS / 1e9 + 200, Geometry)[0]["timestamp"] == CAPTURE_NS / 1e9   # never re-stamped
    edited = copy.deepcopy(good)
    edited["objects"][0]["pose"]["xyz"][0] += 0.001                       # geometry no longer the rendered world
    restamped = copy.deepcopy(good)
    restamped["objects"][0]["timestamp"] += 30.0
    aged = copy.deepcopy(good)
    aged["objects"][0]["age_seconds"] = 0.0
    live = copy.deepcopy(good)
    live["source"]["camera_source"] = "realsense_d435i"
    for bad in (edited, restamped, aged, live):
        with pytest.raises(ValueError, match="RENDER_FRAME_INVALID"):
            rendered_frame(bad)
    assert rendered_frame({"source": {"mode": "replayed_snapshot", "producer": "epd_replay"}}) is None


def test_only_a_fake_hardware_sorting_cycle_takes_a_rendered_frame_and_never_a_live_one():
    from scripts.perceived_object_grasp_execute import sorting_cycle_input
    rendered, cycle = frame(INITIAL, {"cup-1": "cup"}), {"session_id": "r", "cycle": 1}
    live = {"schema_version": "detected_objects/v1", "source": {"mode": "live_epd", "camera_source": "simulated_rgbd"},
            "objects": []}
    assert sorting_cycle_input(rendered, sorting_cycle=cycle, replay=False, backend="fake")["producer"] == RENDER_PRODUCER
    for kwargs in (dict(sorting_cycle=None, replay=False, backend="fake"), dict(sorting_cycle=cycle, replay=True, backend="fake"),
                   dict(sorting_cycle=cycle, replay=False, backend="simulator")):
        with pytest.raises(ValueError, match="RENDER_FRAME_SORTING_ONLY"):
            sorting_cycle_input(rendered, **kwargs)
    with pytest.raises(ValueError, match="SORTING_INPUT_UNSUPPORTED"):
        sorting_cycle_input(live, sorting_cycle=cycle, replay=False, backend="fake")
    assert sorting_cycle_input(live, sorting_cycle=None, replay=False, backend="fake") is None   # ordinary live runs


def test_the_execution_boundary_verifies_the_rendered_world_and_never_renews_it():
    from scripts.perceived_object_grasp_execute import rendered_world_revalidation
    source = rendered_frame(frame(INITIAL, {"cup-1": "cup"}))
    cup = {"id": "runtime::cup-1", "pose": [0.3, -0.1, 0.05, 0, 0, 0, 1], "dimensions": [0.06, 0.05, 0.10],
           "timestamp": CAPTURE_NS / 1e9}
    evidence = rendered_world_revalidation([cup], {cup["id"]: copy.deepcopy(cup)}, source, clock=lambda: CAPTURE_NS / 1e9 + 90)
    assert cup["timestamp"] == CAPTURE_NS / 1e9 and not evidence["timestamps_renewed"] and evidence["synthetic"]
    assert evidence["age_at_boundary_s"] == pytest.approx(90.0) and evidence["world_revision"] == source["world_revision"]
    moved = dict(copy.deepcopy(cup), pose=[0.3, -0.1, 0.06, 0, 0, 0, 1])
    with pytest.raises(RuntimeError, match="SORTING_WORLD_CHANGED"):
        rendered_world_revalidation([cup], {cup["id"]: moved}, source)


def test_freshness_of_a_rendered_frame_is_its_own_and_live_cameras_stay_strict():
    from scripts.task_intent_v2 import (freshness_source_type, observation_age_cap, session_rendered_perception)
    task = routed_intent(RULES)
    task["perception"] = {"mode": "live_epd", "camera_source": "simulated_rgbd"}
    assert freshness_source_type(task) == "simulated_rgbd_fake_hardware" and session_rendered_perception(task)
    assert observation_age_cap("simulated_rgbd_fake_hardware", RENDER_PRODUCER) == 300.0
    assert observation_age_cap("simulated_rgbd_fake_hardware", "live_sensor") == 2.0          # a live capture: unchanged
    for source in ("live_epd_realsense", "recorded_perception", "perception", "replayed_perception_in_simulation"):
        with pytest.raises(ValueError, match="OBSERVATION_PRODUCER_REJECTED"):
            observation_age_cap(source, RENDER_PRODUCER)
    for edit in ({"perception": {"mode": "live_epd", "camera_source": "realsense_d435i"}},
                 {"perception": {"mode": "live_epd", "camera_source": "simulated_rgbd", "replay_path": "x.yaml"}},
                 {"safety": {"execution_backend": "simulator"}}, {"routing": {}}):
        assert not session_rendered_perception(dict(copy.deepcopy(task), **edit))


def test_session_input_is_rendered_by_the_session_and_nothing_else_can_run_it(tmp_path):
    from scripts.task_intent_authoring import SESSION_RENDER_INPUT, planning_input
    task = routed_intent(RULES)
    task["perception"] = {"mode": "live_epd", "camera_source": "simulated_rgbd"}
    assert planning_input(tmp_path, task) == (SESSION_RENDER_INPUT, False)
    with pytest.raises(ValueError, match="TASK_INPUT_OVERRIDE_REJECTED"):
        planning_input(tmp_path, task, tmp_path / "other.yaml")
    task["perception"]["camera_source"] = "realsense_d435i"
    with pytest.raises(ValueError, match="TASK_INPUT_UNAVAILABLE"):
        planning_input(tmp_path, task)


def test_successive_rendered_frames_classify_place_and_never_repick():
    s = session()
    first = s.render_world()
    assert first == INITIAL
    ingest(s, frame(first, {"cup-1": "cup"}))                             # EPD classified only the cup
    assert s.objects["runtime::bowl-1"]["class_id"] == ss.UNPERCEIVED
    target = s.next_target()
    assert target["id"] == "runtime::cup-1"
    # Unclassified objects stay obstacles and are never subject to the unrouted policy (stop_session here).
    assert s.objects["runtime::thing-1"]["status"] == ss.NOT_ELIGIBLE
    assert "UNPERCEIVED" in s.objects["runtime::thing-1"]["reason"]
    s.begin_cycle(target)
    assert s.record_cycle(completed_summary(s, target, [1.0, 2.0, 3.06]), 0, [4] * 9) == ss.COMPLETED
    second = s.render_world()                                              # the cup now rests in its bin
    assert next(o for o in second if o["object_id"] == "cup-1")["pose"]["xyz"] == [1.0, 2.0, 3.06]
    with pytest.raises(ss.SortingFault, match="FRAME_WORLD_MISMATCH"):
        ingest(s, frame(first, {"cup-1": "cup"}))                         # a frame of the old world is never used
    ingest(s, frame(second, {"cup-1": "can", "bowl-1": "box"}))           # EPD now sees bowl-1; the placed cup reads differently
    assert s.objects["runtime::cup-1"]["status"] == ss.COMPLETED and s.objects["runtime::cup-1"]["class_id"] == "cup"
    target = s.next_target()
    assert target["id"] == "runtime::bowl-1"                              # the cup is never picked again
    assert s.observation_status()["frames"] == 2 and s.observation_status()["classified"] == 2


def test_an_object_whose_class_changes_is_never_picked_and_an_unseen_one_waits():
    s = session()
    ingest(s, frame(INITIAL, {"cup-1": "cup", "bowl-1": "box"}))
    ingest(s, frame(INITIAL, {"cup-1": "can"}))                           # cup-1 changed class; bowl-1 unseen now
    target = s.next_target()
    assert target is None
    assert s.objects["runtime::cup-1"]["status"] == ss.NOT_ELIGIBLE and "CLASS_UNSTABLE" in s.objects["runtime::cup-1"]["reason"]
    assert s.objects["runtime::bowl-1"]["status"] == ss.NOT_ELIGIBLE and "UNPERCEIVED" in s.objects["runtime::bowl-1"]["reason"]
    ingest(s, frame(INITIAL, {"cup-1": "cup", "bowl-1": "box"}))          # bowl-1 classified again; cup-1 stays unstable
    assert s.next_target()["id"] == "runtime::bowl-1"
    assert s.objects["runtime::cup-1"]["status"] == ss.NOT_ELIGIBLE


def test_a_rendered_frame_must_reproduce_the_session_world():
    s = session()
    ingest(s, frame(INITIAL, {"cup-1": "cup"}))
    moved = copy.deepcopy(INITIAL)
    moved[0]["pose"]["xyz"][0] += 0.01
    with pytest.raises(ss.SortingFault, match="FRAME_WORLD_MISMATCH"):
        ingest(s, frame(moved, {"cup-1": "cup"}, revision=world_revision(INITIAL)))


def test_session_frame_takes_classes_only_from_epd_and_geometry_only_from_the_world(tmp_path, monkeypatch):
    """perception_replay.session_frame with the renderer and EPD replaced by their recorded outputs."""
    import json
    import yaml
    from scripts import perception_replay as pr
    (tmp_path / "environment.yaml").write_text(yaml.safe_dump(
        {"camera": {"camera_id": "cam", "frame_id": "opt", "pose": [0.0] * 6}}))
    world_now = world(("cup-1", 0.5, 0.0, 0.05, (0.06, 0.05, 0.10)), ("bowl-1", 0.5, 0.3, 0.05, (0.06, 0.05, 0.10)))
    import simulator_world                                  # the module session_frame imports (scripts on sys.path)
    monkeypatch.setattr(simulator_world, "derive_render_world", lambda scene: (
        "<sdf/>", {"schema": "workcell_render_world/v1",
                   "objects": [{"object_id": o["object_id"], "dimensions_m": o["dimensions"]} for o in world_now]}))
    rendered = {}

    def record(scene, output, **kwargs):
        rendered.update(kwargs)
        output.mkdir(parents=True)
        manifest = {"frames": [{"stamp_ns": 1, "received_wall_ns": CAPTURE_NS - 200_000_000},
                               {"stamp_ns": 2, "received_wall_ns": CAPTURE_NS}],
                    "cell": str(scene), "physics_world_sha256": None, "render_world_sha256": "r",
                    "calibration": {"status": pr.SIMULATION_EXACT, "camera_id": "cam", "camera_link_pose_world": [0.0] * 6,
                                    "optical_in_link": [list(row) for row in pr.OPTICAL_IN_LINK]}}
        (output / "recording.json").write_text(json.dumps(manifest))
        return manifest

    def optical(x, y, z):                                   # camera at the world origin looking along +x
        return [-y, -z, x]
    epd = [{"runtime_mode": "replay", "timestamp_ns": 2, "camera_id": "cam", "frame_id": "opt", "rejected_objects": [],
            "objects": [{"object_id": "t1", "label": "cup", "confidence": 0.9, "dimensions_xyz": [0.05, 0.05, 0.08],
                         "pose": {"frame_id": "opt", "position": optical(0.5, 0.0, 0.06)}},
                        {"object_id": "t2", "label": "bottle", "confidence": 0.7, "dimensions_xyz": [0.05, 0.05, 0.2],
                         "pose": {"frame_id": "opt", "position": optical(0.9, -0.4, 0.1)}}]}]
    monkeypatch.setattr(pr, "record", record)
    monkeypatch.setattr(pr, "run_epd_replay", lambda *a, **k: (epd, {"performance": {}}))
    snapshot = pr.session_frame(tmp_path, tmp_path / "frame", tmp_path, world_now, world_revision(world_now))
    assert rendered["poses"] == {"cup-1": [0.5, 0.0, 0.05, 0.0, 0.0, 0.0], "bowl-1": [0.5, 0.3, 0.05, 0.0, 0.0, 0.0]}
    by_id = {o["object_id"]: o for o in snapshot["objects"]}
    assert (by_id["cup-1"]["class_id"], by_id["cup-1"]["confidence"]) == ("cup", 0.9)
    assert by_id["bowl-1"]["class_id"] == "unperceived" and by_id["bowl-1"]["confidence"] is None   # nothing invented
    assert by_id["cup-1"]["dimensions"] == [0.06, 0.05, 0.10]                      # world geometry, not EPD's extent
    assert [u["class_id"] for u in snapshot["source"]["unbound_detections"]] == ["bottle"]
    assert rendered_frame(snapshot)["capture_wall_ns"] == CAPTURE_NS                # the classified frame's capture
    assert yaml.safe_load((tmp_path / "frame" / "frame.yaml").read_text()) == snapshot
    with pytest.raises(pr.PerceptionBlocked, match="WORLD_REVISION_MISMATCH"):
        pr.session_frame(tmp_path, tmp_path / "f2", tmp_path, world_now, "0" * 64)
    with pytest.raises(pr.PerceptionBlocked, match="WORLD_OBJECTS_MISMATCH"):
        pr.session_frame(tmp_path, tmp_path / "f3", tmp_path, world_now[:1], world_revision(world_now[:1]))
