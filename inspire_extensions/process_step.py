from dataclasses import dataclass, field

import numpy as np

from incar.preprocessing import PreProcessStep
from incar.common import ProcessHook

from .retargeting import Retargeter

@PreProcessStep.register_subclass("inspire_hand_remapping")
@dataclass
class InspireHandRemapping(PreProcessStep):
    hooks: list[ProcessHook] = field(default_factory = lambda: [ProcessHook.TELEOP_COMMAND])
    hand_features: list[str] = field(default_factory = lambda: ["left.commands.hand.joints.position", "right.commands.hand.joints.position"])
    processed_features: list[str] = field(default_factory = lambda: ["left.commands.hand.inspire", "right.commands.hand.inspire"])

    def __post_init__(self):
        self.retargeter = Retargeter()

    def process_single_frame(self, frame: dict):
        # prevent foreach loop where keys of dict change during loop
        keys: list[str] = []
        for key in frame.keys():
            if not key in self.hand_features:
                continue 
            keys.append(key)

        for key in keys:
            goal_angles = self.retargeter.map_joints_to_goal_angles(frame[key])

            scaling_factors = [1.4,1.3,1.3,1.3,4,5]
            if key.startswith("right"):
                offset_factors = [0,0,0,0,0,1.4]
                goal_angles[5] = -goal_angles[5]

            if key.startswith("left"):
                offset_factors = [0,0,0,0,0,-1.7]
            goal_angles = np.add(goal_angles, offset_factors)
            goal_angles = np.multiply(goal_angles, scaling_factors)

            # Check if angles are within the range [0, π]
            for i, angle in enumerate(goal_angles):
                if angle < 0 or angle > np.pi:
                    goal_angles[i] = np.clip(angle, 0, np.pi)

            # Convert to new scale (assuming radians to degrees or another scale)
            goal_angles = np.multiply(goal_angles, np.pi*100)
            
            # Adjust goal angles by subtracting from 1000
            goal_angles = 1000 - np.round(goal_angles)

            corresponding_processed_key = self.processed_features[self.hand_features.index(key)]
            frame[corresponding_processed_key] = goal_angles.tolist()