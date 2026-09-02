from __future__ import annotations

from dataclasses import dataclass
import base64
import io
import numpy as np
from PIL import Image

from vector_os_nano.integrations import navigation_models

from vector_os_nano import Agent
from vector_os_nano.integrations.online_mapper import RGBDObservation
from vector_os_nano.integrations.online_navigation import OnlineNavigationConfig, OnlineNavigator
from vector_os_nano.navigation.trajectory_planners import AStarTrajectoryPlanner
from vector_os_nano.navigation.embodiment import MolmoSpacesRBY1NavigationEmbodiment
from vector_os_nano.core.skill import SkillContext
from vector_os_nano.skills.navigation_vln import (
    OneRINGNavigationSkill, _navigation_config, _normalize_navigation_target,
    _onering_instruction,
)


class Base:
    name = "mobile"
    def get_position(self): return [0.0, 0.0, 0.0]
    def get_heading(self): return 0.0
    def get_rgbd_frame(self, width, height): return np.zeros((height, width, 3), np.uint8), np.ones((height, width), np.float32)
    def walk(self, **kwargs): return True


def observation():
    return RGBDObservation(rgb=np.zeros((24, 32, 3), np.uint8), depth=np.ones((24, 32), np.float32), intrinsics=np.array([[20, 0, 16], [0, 20, 12], [0, 0, 1]], np.float32), camera_pose=np.eye(4), base_pose=np.zeros(3))


def test_default_agent_registers_only_onering_and_astar_navigation() -> None:
    names = set(Agent(base=Base()).skills)
    assert {"onering_navigation", "astar_plan"} <= names


def test_rby1_navigation_uses_molmospaces_footprint_radius() -> None:
    context = SkillContext(services={"molmospaces_rby1": {}})
    assert _navigation_config(context)["astar"]["robot_radius_m"] == 0.05


def test_onering_instruction_uses_objectnav_sentence() -> None:
    assert _onering_instruction("green trash can") == "Go to the green trash can."
    assert _onering_instruction("the refrigerator") == "Go to the refrigerator."
    assert _onering_instruction("Navigate to the chair") == "Navigate to the chair."
    assert _onering_instruction("find the sofa!") == "find the sofa."


def test_chinese_object_navigation_target_is_normalized() -> None:
    assert _normalize_navigation_target("导航去冰箱那边") == "refrigerator"
    assert _normalize_navigation_target("请你走到绿色垃圾桶旁边") == "green trash can"


def test_rby1_strategy_selector_routes_object_navigation_to_onering() -> None:
    from vector_os_nano.core.skill import SkillRegistry
    from vector_os_nano.vcli.cognitive.strategy_selector import StrategySelector
    from vector_os_nano.vcli.cognitive.types import SubGoal

    registry = SkillRegistry(); registry.register(OneRINGNavigationSkill())
    selector = StrategySelector(skill_registry=registry, has_base=True)
    sub_goal = SubGoal(
        name="navigate_to_fridge", description="导航去冰箱那边", verify="True",
        strategy="navigate_skill", strategy_params={"room": "fridge"},
    )

    result = selector.select(sub_goal)

    assert result.name == "onering_navigation"
    assert result.params == {"target": "fridge"}


def test_local_astar_returns_base_local_path() -> None:
    planner = AStarTrajectoryPlanner(); planner.reset(observation().intrinsics)
    result = planner.plan(observation(), mode="pointgoal", goal=[1.5, 0.3])
    assert result["planner"] == "astar"
    assert result["trajectory_frame"] == "base_local"
    assert result["trajectory"]


def test_astar_floor_filter_keeps_free_space_and_marks_vertical_obstacle() -> None:
    intrinsics = np.array([[80, 0, 160], [0, 80, 120], [0, 0, 1]], np.float32)
    camera_pose = np.array([
        [0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 1], [0, 0, 0, 1],
    ], dtype=np.float32)
    depth = np.zeros((240, 320), np.float32)
    depth[200:232, :] = 1.0  # floor samples at approximately world z=0
    depth[80:160, 144:176] = 1.0  # vertical object around world z=1
    obs = RGBDObservation(np.zeros((240, 320, 3), np.uint8), depth, intrinsics, camera_pose, np.zeros(3))
    planner = AStarTrajectoryPlanner(); planner.reset(intrinsics)

    grid = planner._occupancy(obs)

    assert 0 < int(grid.sum()) < grid.size // 2
    result = planner.plan(obs, mode="nogoal", goal=None)
    assert abs(result["occupancy_grid"]["occupied_cells"] - int(grid.sum())) <= 2


def test_astar_plans_on_exported_global_occupancy_map() -> None:
    free = np.ones((100, 100), dtype=np.uint8) * 255
    free[50, :] = 0
    free[50, 45:55] = 255
    buffer = io.BytesIO(); Image.fromarray(free).save(buffer, format="PNG")
    payload = {
        "success": True, "data": base64.b64encode(buffer.getvalue()).decode(),
        "shape": [100, 100], "px_per_m": 10.0, "robot_radius_m": 0.05,
        "world_to_map": [[10, 0, 0, 10], [0, 10, 0, 10]],
        "map_to_world": [[0.1, 0, -1], [0, 0.1, -1], [0, 0, 0]],
    }
    planner = AStarTrajectoryPlanner(); planner.reset(observation().intrinsics)

    metadata = planner.set_global_map(payload)
    result = planner.plan_global(observation(), np.array([5.0, 5.0]))

    assert metadata["shape"] == [100, 100]
    assert result["trajectory_frame"] == "world"
    assert result["trajectory"]
    assert result["global_path_cells"] > 0


def test_astar_reset_discards_previous_global_scene_map() -> None:
    planner = AStarTrajectoryPlanner()
    planner.reset(observation().intrinsics)
    planner._global_free = np.ones((4, 4), dtype=bool)
    planner._global_world_to_map = np.eye(3)
    planner._global_map_to_world = np.eye(3)
    planner._global_metadata = {"source": "previous_scene"}
    planner.reset(observation().intrinsics)
    assert planner.has_global_map is False
    assert planner._global_metadata == {}


class Embodiment:
    name = "fake"
    def __init__(self): self.actions = []
    def ensure_ready(self): return {"loaded": True}
    def prepare_navigation(self, target): return {"prepared": True, "target": target}
    def observe_rgbd(self): return observation()
    def execute_discrete_action(self, action, _observation, **_kwargs): self.actions.append(action); return {"success": True}
    def execute_local_waypoint(self, _point, _observation, **_kwargs): self.actions.append("astar"); return {"success": True}
    def check_navigation_goal(self, *_args, **_kwargs): return {"known": False, "reached": False}
    def stop(self): pass


class Ring:
    def reset(self, _instruction): return {"ok": True}
    def step(self, **_kwargs): return {"action": "move_ahead", "action_index": 0}


class BrokenRing:
    def reset(self, _instruction): return {"ok": True}
    def step(self, **_kwargs): return {"action": "pickup", "action_index": 8}


class Mapper:
    def __init__(self): self.update_calls = 0
    def update(self, *_args, **_kwargs): self.update_calls += 1; return {"recorded": True}
    def close(self): pass


class Graph:
    def find_objects_by_category(self, _target): return []


def test_unknown_target_uses_onering_without_planner() -> None:
    embodiment = Embodiment()
    mapper = Mapper()
    navigator = OnlineNavigator(embodiment=embodiment, planner=AStarTrajectoryPlanner(), onering=Ring(), mapper=mapper, scene_graph=Graph(), config=OnlineNavigationConfig(max_steps=1))
    result = navigator.navigate("chair")
    assert embodiment.actions == ["move_ahead"]
    assert result["planning_sources"]["onering"] == 1
    assert result["trajectory_planners"] == {}
    assert mapper.update_calls == 0
    assert result["mapping"][0]["reason"] == "deferred_during_navigation_control"


def test_onering_client_encodes_shared_rby1_frame_once(monkeypatch) -> None:
    encoded = []
    payloads = []

    monkeypatch.setattr(
        navigation_models,
        "_png_rgb",
        lambda image: encoded.append(image) or "png",
    )
    monkeypatch.setattr(
        navigation_models,
        "_post_json",
        lambda _url, payload, _timeout: payloads.append(payload) or {"action": "move_ahead"},
    )

    frame = np.zeros((8, 10, 3), dtype=np.uint8)
    client = navigation_models.OneRINGClient()
    client.step(
        navigation_rgb=frame,
        manipulation_rgb=None,
        instruction="Go to the chair.",
    )

    assert encoded == [frame]
    assert "manipulation_rgb_png" not in payloads[0]


def test_astar_goal_does_not_substring_match_winebottle_for_bottle_target() -> None:
    from types import SimpleNamespace

    class Graph:
        def find_objects_by_category(self, _target):
            return [SimpleNamespace(category="winebottle", description="wine bottle", x=9.0, y=9.0)]

    navigator = OnlineNavigator.__new__(OnlineNavigator)
    navigator.scene_graph = Graph()
    assert navigator._known_goal("bottle", np.zeros(3)) is None


def test_three_consecutive_onering_failures_trigger_one_global_sync_then_astar() -> None:
    class FallbackEmbodiment(Embodiment):
        def __init__(self): super().__init__(); self.syncs = 0
        def global_scene_sync(self, _graph): self.syncs += 1; return {"objects": 1}

    class Planner:
        name = "astar"
        def reset(self, _intrinsics): return {"ok": True}
        def plan(self, _obs, *, mode, goal): return {"planner": "astar", "trajectory": [[1.0, 0.0]], "trajectory_frame": "base_local", "mode": mode, "goal": goal}

    embodiment = FallbackEmbodiment()
    navigator = OnlineNavigator(embodiment=embodiment, planner=Planner(), onering=BrokenRing(), mapper=Mapper(), scene_graph=Graph(), config=OnlineNavigationConfig(max_steps=4))
    result = navigator.navigate("chair")

    assert embodiment.syncs == 1
    assert result["trajectory_planners"]["astar"] >= 1


def test_rby1_waypoint_retries_lazy_first_failure_and_verifies_motion() -> None:
    class Bridge:
        def __init__(self): self.waypoint_calls = 0
        def execute(self, _instruction, *, context, mode, timeout_s):
            del mode, timeout_s
            if context["structured_action"] != "step_waypoint":
                raise RuntimeError("warmup observation unavailable in fake")
            self.waypoint_calls += 1
            if self.waypoint_calls == 1:
                return {"success": False, "error": "lazy nav stack not ready"}
            return {"success": True, "state": {"robot": {"base_pose": [0.2, 0, 0, 1, 0, 0, 0]}}}

    bridge = Bridge()
    embodiment = MolmoSpacesRBY1NavigationEmbodiment(
        bridge, {}, camera="head_camera", timeout_s=5.0,
    )

    result = embodiment._execute_world_waypoint(
        [0.2, 0.0, 0.0], observation(), control_steps=1,
        instruction="test waypoint", waypoint_max_attempts=2,
    )

    assert result["success"] is True
    assert result["verified_motion"] is True
    assert result["execution_attempts"] == 2
    assert bridge.waypoint_calls == 2


def test_rby1_prepare_navigation_does_not_query_oracle_targets() -> None:
    class Bridge:
        def execute(self, instruction, *, context, mode, timeout_s):
            assert instruction == "navigate to the refrigerator"
            assert context["structured_action"] == "prepare_navigation"
            assert context["target_types"] == ["refrigerator"]
            return {"prepared": True, "target_object": "refrigerator_1"}

    embodiment = MolmoSpacesRBY1NavigationEmbodiment(
        Bridge(), {}, camera="head_camera", timeout_s=5.0,
    )

    assert embodiment.prepare_navigation("refrigerator")["external_navigation_target"] == "refrigerator"


def test_rby1_rotation_waits_until_full_discrete_angle() -> None:
    import math

    class Bridge:
        def __init__(self): self.calls = 0
        def execute(self, _instruction, *, context, mode, timeout_s):
            del mode, timeout_s
            if context["structured_action"] != "step_waypoint":
                raise RuntimeError("no observation in fake")
            self.calls += 1
            yaw = [0.10, 0.30, math.radians(30.0)][min(self.calls - 1, 2)]
            return {"success": True, "state": {"robot": {"base_pose": [
                0, 0, 0, math.cos(yaw / 2), 0, 0, math.sin(yaw / 2),
            ]}}}

    bridge = Bridge()
    embodiment = MolmoSpacesRBY1NavigationEmbodiment(
        bridge, {}, camera="head_camera", timeout_s=5.0,
    )
    result = embodiment._execute_world_waypoint(
        [0.0, 0.0, math.radians(30.0)], observation(), control_steps=1,
        instruction="rotate left", waypoint_max_attempts=5,
    )

    assert result["success"] is True
    assert result["execution_attempts"] == 3
    assert result["yaw_error_rad"] < math.radians(2.0)
