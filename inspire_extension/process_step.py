from dataclasses import dataclass, field

import numpy as np

from incar.extensions.processing_step import ProcessStep
from incar.common import ProcessHook

from .retargeting import Retargeter


@ProcessStep.register_subclass("inspire_hand_remapping")
@dataclass
class InspireHandRemapping(ProcessStep):
    """Full 6DOF hand teleoperation — maps Quest hand tracking joints to Inspire hand angles."""
    hooks: list[ProcessHook] = field(default_factory=lambda: [ProcessHook.TELEOP_COMMAND])
    hand_features: list[str] = field(default_factory=lambda: ["left.commands.hand.joints.position", "right.commands.hand.joints.position"])
    processed_features: list[str] = field(default_factory=lambda: ["left.commands.hand.inspire", "right.commands.hand.inspire"])

    def __post_init__(self):
        self.retargeter = Retargeter()

    def process_single_frame(self, frame: dict):
        keys = [k for k in frame if k in self.hand_features]
        for key in keys:
            goal_angles = self.retargeter.map_joints_to_goal_angles(frame[key])

            scaling_factors = [1.4, 1.3, 1.3, 1.3, 4, 5]
            if key.startswith("right"):
                offset_factors = [0, 0, 0, 0, 0, 1.4]
                goal_angles[5] = -goal_angles[5]
            if key.startswith("left"):
                offset_factors = [0, 0, 0, 0, 0, -1.7]

            goal_angles = np.add(goal_angles, offset_factors)
            goal_angles = np.multiply(goal_angles, scaling_factors)
            goal_angles = np.clip(goal_angles, 0, np.pi)
            goal_angles = np.multiply(goal_angles, np.pi * 100)
            goal_angles = 1000 - np.round(goal_angles)

            out_key = self.processed_features[self.hand_features.index(key)]
            frame[out_key] = goal_angles.tolist()


@ProcessStep.register_subclass("inspire_hand_gripper")
@dataclass
class InspireHandGripper(ProcessStep):
    """
    1DOF open/close gripper control — maps a single binary signal to two configurable
    Inspire hand poses. Use a registered subclass for preset grasps, or set open_pose
    and close_pose directly for a custom grasp.

    Finger order: [pinky, ring, middle, index, thumb_flex, thumb_rot]
    Angle range:  0 (fully open) – 1000 (fully closed), unit: 1/10 degree
    """
    hooks: list[ProcessHook] = field(default_factory=lambda: [ProcessHook.TELEOP_COMMAND, ProcessHook.INFERENCE_COMMAND])
    gripper_features: list[str] = field(default_factory=lambda: ["left.commands.gripper.openclose", "right.commands.gripper.openclose"])
    processed_features: list[str] = field(default_factory=lambda: ["left.commands.hand.inspire", "right.commands.hand.inspire"])

    open_pose:  list[int] = field(default_factory=lambda: [1740, 1740, 1740, 1740, 1350, 1700])
    close_pose: list[int] = field(default_factory=lambda: [900,  900,  900,  900,  1100,  800])
    threshold: float = 0.5

    def process_single_frame(self, frame: dict):
        for key in [k for k in frame if k in self.gripper_features]:
            pose = self.close_pose if frame[key][0] >= self.threshold else self.open_pose
            out_key = self.processed_features[self.gripper_features.index(key)]
            frame[out_key] = pose


# Example of overloading with custom pose
@ProcessStep.register_subclass("inspire_gripper_example_pose")
@dataclass
class InspireGripperExamplePose(InspireHandGripper):
    """
    Preset for basketbal grasp
    """
    open_pose:  list[int] = field(default_factory=lambda: [900,  900,  1800, 1800, 1450, 850])
    close_pose: list[int] = field(default_factory=lambda: [900,  900,  1400,  1400,  1250,  850])
