# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2026 Vector Robotics

"""Tests for redesigned SkillContext with dict registries."""
import pytest
from unittest.mock import MagicMock
from vector_os_nano.core.skill import SkillContext
from vector_os_nano.core.world_model import WorldModel


class TestSkillContextRegistries:
    def test_empty_context(self):
        ctx = SkillContext(world_model=WorldModel())
        assert ctx.arm is None
        assert ctx.gripper is None
        assert ctx.hand is None
        assert ctx.base is None
        assert ctx.perception is None

    def test_single_arm(self):
        arm = MagicMock()
        ctx = SkillContext(arms={"so101": arm}, world_model=WorldModel())
        assert ctx.arm is arm
        assert ctx.has_arm()
        assert ctx.has_arm("so101")
        assert not ctx.has_arm("other")

    def test_single_base(self):
        base = MagicMock()
        ctx = SkillContext(bases={"go2": base}, world_model=WorldModel())
        assert ctx.base is base
        assert ctx.has_base()
        assert ctx.has_base("go2")

    def test_multiple_arms(self):
        arm1 = MagicMock()
        arm2 = MagicMock()
        ctx = SkillContext(arms={"so101": arm1, "ur5": arm2}, world_model=WorldModel())
        assert ctx.arm is arm1  # first one
        assert ctx.has_arm("ur5")
        assert ctx.get_arm("ur5") is arm2

    def test_explicit_defaults_override_registry_order(self):
        left_arm, right_arm = MagicMock(), MagicMock()
        left_gripper, right_gripper = MagicMock(), MagicMock()
        left_hand, right_hand = MagicMock(), MagicMock()
        base_a, base_b = MagicMock(), MagicMock()
        ctx = SkillContext(
            arms={"left": left_arm, "right": right_arm},
            grippers={"left": left_gripper, "right": right_gripper},
            hands={"left": left_hand, "right": right_hand},
            bases={"primary": base_a, "backup": base_b},
            default_arm_name="right",
            default_gripper_name="right",
            default_hand_name="right",
            default_base_name="backup",
        )

        assert ctx.arm is right_arm
        assert ctx.gripper is right_gripper
        assert ctx.hand is right_hand
        assert ctx.base is base_b
        assert ctx.get_arm() is right_arm
        assert ctx.get_gripper() is right_gripper
        assert ctx.get_hand() is right_hand
        assert ctx.get_base() is base_b

    @pytest.mark.parametrize(
        ("registry_kw", "default_kw"),
        [
            ("arms", "default_arm_name"),
            ("grippers", "default_gripper_name"),
            ("hands", "default_hand_name"),
            ("bases", "default_base_name"),
        ],
    )
    def test_unknown_explicit_default_fails_fast(self, registry_kw, default_kw):
        with pytest.raises(ValueError, match="Unknown default"):
            SkillContext(**{registry_kw: {"left": MagicMock()}, default_kw: "right"})

    def test_named_hands_queries(self):
        left, right = MagicMock(), MagicMock()
        ctx = SkillContext(
            hands={"left": left, "right": right},
            default_hand_name="right",
        )

        assert ctx.has_hand()
        assert ctx.has_hand("left")
        assert not ctx.has_hand("missing")
        assert ctx.get_hand("left") is left
        assert ctx.get_hand("missing") is None
        assert ctx.hand is right
        assert "hands=['left', 'right']" in repr(ctx)

    def test_no_base(self):
        ctx = SkillContext(world_model=WorldModel())
        assert not ctx.has_base()
        assert ctx.base is None
        assert ctx.get_base("go2") is None

    def test_capabilities(self):
        arm = MagicMock()
        base = MagicMock()
        ctx = SkillContext(
            arms={"so101": arm},
            hands={"left": MagicMock()},
            bases={"go2": base},
            world_model=WorldModel(),
        )
        caps = ctx.capabilities()
        assert caps["has_arm"] is True
        assert caps["has_base"] is True
        assert caps["has_gripper"] is False
        assert caps["has_hand"] is True
        assert "so101" in caps["arm_names"]
        assert caps["hand_names"] == ["left"]
        assert "go2" in caps["base_names"]

    def test_services_registry(self):
        nav = MagicMock()
        ctx = SkillContext(services={"nav": nav}, world_model=WorldModel())
        assert ctx.services["nav"] is nav

    def test_perception_sources(self):
        cam = MagicMock()
        ctx = SkillContext(perception_sources={"realsense": cam}, world_model=WorldModel())
        assert ctx.perception is cam
        assert ctx.has_perception()

    def test_get_arm_default(self):
        arm = MagicMock()
        ctx = SkillContext(arms={"a": arm}, world_model=WorldModel())
        assert ctx.get_arm() is arm
        assert ctx.get_arm("a") is arm
        assert ctx.get_arm("nonexistent") is None

    def test_get_base_default(self):
        base = MagicMock()
        ctx = SkillContext(bases={"b": base}, world_model=WorldModel())
        assert ctx.get_base() is base

    def test_backward_compat_config(self):
        ctx = SkillContext(config={"key": "val"}, world_model=WorldModel())
        assert ctx.config["key"] == "val"

    def test_calibration_field(self):
        cal = MagicMock()
        ctx = SkillContext(calibration=cal, world_model=WorldModel())
        assert ctx.calibration is cal


class TestSkillContextBackwardCompat:
    """Ensure existing skill code that uses context.arm / context.base still works."""

    def test_walk_skill_pattern(self):
        """WalkSkill does: context.base.walk(vx, vy, vyaw, dur)"""
        base = MagicMock()
        base.walk.return_value = True
        ctx = SkillContext(bases={"go2": base}, world_model=WorldModel())
        # Simulates what WalkSkill does
        assert ctx.base is not None
        result = ctx.base.walk(0.3, 0, 0, 2.0)
        assert result is True

    def test_pick_skill_pattern(self):
        """PickSkill does: context.arm, context.gripper, context.perception, context.calibration"""
        arm = MagicMock()
        gripper = MagicMock()
        perception = MagicMock()
        cal = MagicMock()
        ctx = SkillContext(
            arms={"so101": arm},
            grippers={"so101": gripper},
            perception_sources={"realsense": perception},
            calibration=cal,
            world_model=WorldModel(),
        )
        assert ctx.arm is arm
        assert ctx.gripper is gripper
        assert ctx.perception is perception
        assert ctx.calibration is cal

    def test_legacy_flat_kwargs_arm(self):
        """Old-style arm= kwarg populates arm property via legacy fallback."""
        arm = MagicMock()
        ctx = SkillContext(arm=arm, world_model=WorldModel())
        assert ctx.arm is arm

    def test_legacy_flat_kwargs_base(self):
        """Old-style base= kwarg populates base property via legacy fallback."""
        base = MagicMock()
        ctx = SkillContext(base=base, world_model=WorldModel())
        assert ctx.base is base

    def test_legacy_flat_kwargs_gripper(self):
        """Old-style gripper= kwarg populates gripper property via legacy fallback."""
        gripper = MagicMock()
        ctx = SkillContext(gripper=gripper, world_model=WorldModel())
        assert ctx.gripper is gripper

    def test_legacy_flat_kwargs_perception(self):
        """Old-style perception= kwarg populates perception property via legacy fallback."""
        perception = MagicMock()
        ctx = SkillContext(perception=perception, world_model=WorldModel())
        assert ctx.perception is perception

    def test_dict_registry_takes_priority_over_legacy(self):
        """Dict registry arm takes priority over legacy arm= kwarg."""
        legacy_arm = MagicMock()
        dict_arm = MagicMock()
        ctx = SkillContext(
            arm=legacy_arm,
            arms={"so101": dict_arm},
            world_model=WorldModel(),
        )
        assert ctx.arm is dict_arm

    def test_legacy_none_perception(self):
        """Legacy perception=None yields perception property None."""
        ctx = SkillContext(
            arm=MagicMock(),
            gripper=MagicMock(),
            perception=None,
            world_model=WorldModel(),
            calibration=None,
        )
        assert ctx.perception is None

    def test_has_arm_with_legacy(self):
        """has_arm() returns True when arm is provided via legacy kwarg."""
        arm = MagicMock()
        ctx = SkillContext(arm=arm, world_model=WorldModel())
        assert ctx.has_arm()

    def test_has_base_with_legacy(self):
        """has_base() returns True when base is provided via legacy kwarg."""
        base = MagicMock()
        ctx = SkillContext(base=base, world_model=WorldModel())
        assert ctx.has_base()

    def test_has_gripper_with_legacy(self):
        """has_gripper() returns True when gripper is provided via legacy kwarg."""
        gripper = MagicMock()
        ctx = SkillContext(gripper=gripper, world_model=WorldModel())
        assert ctx.has_gripper()
