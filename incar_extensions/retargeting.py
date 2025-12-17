import numpy as np
from copy import deepcopy

from incar.messages.teleop_command_pb2 import HandJoints

from .tf_transformations import quaternion_from_matrix, quaternion_matrix

ROBOT_JOINTS_RAW = {
    'index': [3],
    'middle': [2],
    'ring': [1],
    'little': [0],
    'thumb': [4, 5]
}

OCULUS_JOINTS = {
    'wrist': [0],
    'palm': [1],
    'thumb': [2, 3, 4, 5],
    'index': [7, 8, 9, 10],
    'middle': [12, 13, 14, 15],
    'ring': [17, 18, 19, 20],
    'little': [22, 23, 24, 25],
    'metacarpals': [2, 6, 11, 16, 21],
    'knuckles': [7, 12, 17, 22],
}

ROBOT_KEYPOINTS_COUNT = 6
OCULUS_NUM_KEYPOINTS = 26

def normalize_vector(vector):
    return vector / np.linalg.norm(vector)


def calculate_angle(coord_1, coord_2, coord_3):
    vector_1 = coord_2 - coord_1
    vector_2 = coord_3 - coord_2

    inner_product = np.inner(vector_1, vector_2)
    norm = np.linalg.norm(vector_1) * np.linalg.norm(vector_2)
    angle = np.arccos(inner_product / norm)
    return angle

def calculate_angle_z(coord_1, coord_2, coord_3):
    # Project coordinates onto the XY plane by ignoring the Z component
    vector_1 = np.array([coord_2[0] - coord_1[0], coord_2[1] - coord_1[1], 0])
    vector_2 = np.array([coord_3[0] - coord_2[0], coord_3[1] - coord_2[1], 0])

    inner_product = np.inner(vector_1, vector_2)
    norm = np.linalg.norm(vector_1) * np.linalg.norm(vector_2)
    angle = np.arccos(inner_product / norm)
    return angle

class Retargeter():
    def map_joints_to_goal_angles(self, msg: HandJoints):
        finger_coords_array = []
        finger_orientation_array = []

        for i in range(OCULUS_NUM_KEYPOINTS):
            finger_coords_array.append(self.position_to_array(msg.joints[i].pose.position)) # +1 because msg.joints is a dict, and key [0] is for unknown. Joints start from [1]  
        self.finger_coords_array_np = np.array(finger_coords_array)
        for i in range(OCULUS_NUM_KEYPOINTS):
            finger_orientation_array.append(self.orientation_to_array(msg.joints[i].pose.rotation)) # +1 because msg.joints is a dict, and key [0] is for unknown. Joints start from [1]  
        self.finger_orientation_array_np = np.array(finger_orientation_array)

        self.transform_keypoints()
        self.finger_coords = dict(
            wrist=self.finger_coords_array_np[OCULUS_JOINTS['wrist']],
            index = self.finger_coords_array_np[OCULUS_JOINTS['index']],
            middle = self.finger_coords_array_np[OCULUS_JOINTS['middle']],
            ring = self.finger_coords_array_np[OCULUS_JOINTS['ring']],
            little = self.finger_coords_array_np[OCULUS_JOINTS['little']],
            thumb = self.finger_coords_array_np[OCULUS_JOINTS['thumb']],
            metacarpals = self.finger_coords_array_np[OCULUS_JOINTS['metacarpals']],
        )

        goal_angles_arr = []

        goal_angles_arr.append(self.calculate_finger_angles('little', self.finger_coords['little'], self.finger_coords['metacarpals']))
        goal_angles_arr.append(self.calculate_finger_angles('ring', self.finger_coords['ring'], self.finger_coords['metacarpals']))
        goal_angles_arr.append(self.calculate_finger_angles('middle', self.finger_coords['middle'], self.finger_coords['metacarpals']))
        goal_angles_arr.append(self.calculate_finger_angles('index', self.finger_coords['index'], self.finger_coords['metacarpals']))
        goal_angles_arr.append(self.calculate_thumb_angles(self.finger_coords['thumb'])[0])
        goal_angles_arr.append(self.calculate_thumb_angles(self.finger_coords['thumb'])[1])
        
        return goal_angles_arr

    def _translate_coords(self, coords):
        return deepcopy(coords) - coords[12]
        
    def _get_coord_frame(self, wrist_coord, index_knuckle_coord, little_knuckle_coord):
        z_axis = normalize_vector(wrist_coord)
        x_axis = normalize_vector(index_knuckle_coord - little_knuckle_coord)
        y_axis = normalize_vector(np.cross(z_axis, x_axis))
        
        return [y_axis, x_axis, -z_axis] # Change from left-handed unity system to right-handed
    


    def transform_keypoints(self):
        self.finger_coords_array_np = self._translate_coords(self.finger_coords_array_np)
        original_coord_frame = self._get_coord_frame(
            self.finger_coords_array_np[OCULUS_JOINTS['wrist'][0]],
            self.finger_coords_array_np[OCULUS_JOINTS['index'][0]],
            self.finger_coords_array_np[OCULUS_JOINTS['ring'][0]]
        )

        if np.linalg.det(original_coord_frame) == 0:
            print("Original coord frame is singular and cannot be inverted")
            return

        try:
            # Compute the rotation matrix
            rotation_matrix = np.linalg.solve(original_coord_frame, np.eye(3)).T

            # Transform positions
            finger_coords = (rotation_matrix @ self.finger_coords_array_np.T).T

            # Transform orientations
            transformed_orientations = []
            for quaternion in self.finger_orientation_array_np:
                rotation_matrix_quat = quaternion_matrix(quaternion)[:3, :3]
                transformed_matrix = rotation_matrix @ rotation_matrix_quat
                full_transform_matrix = np.eye(4)
                full_transform_matrix[:3, :3] = transformed_matrix
                transformed_quat = quaternion_from_matrix(full_transform_matrix)
                transformed_orientations.append(transformed_quat)

            self.finger_coords_array_np = finger_coords
            self.finger_orientation_array_np = np.array(transformed_orientations)
        except np.linalg.LinAlgError as e:
            print(f"Error computing rotation matrix: {e}")

        # Update finger coordinate dictionary
        self.finger_coords = dict(
            wrist=self.finger_coords_array_np[OCULUS_JOINTS['wrist']],
            thumb=self.finger_coords_array_np[OCULUS_JOINTS['thumb']],
            index=self.finger_coords_array_np[OCULUS_JOINTS['index']],
            middle=self.finger_coords_array_np[OCULUS_JOINTS['middle']],
            ring=self.finger_coords_array_np[OCULUS_JOINTS['ring']],
            metacarpals=self.finger_coords_array_np[OCULUS_JOINTS['metacarpals']],
        )
        # Update finger orientation dictionary
        self.finger_orientations = dict(
            wrist=self.finger_orientation_array_np[OCULUS_JOINTS['wrist']],
            thumb=self.finger_orientation_array_np[OCULUS_JOINTS['thumb']],
            index=self.finger_orientation_array_np[OCULUS_JOINTS['index']],
            middle=self.finger_orientation_array_np[OCULUS_JOINTS['middle']],
            ring=self.finger_orientation_array_np[OCULUS_JOINTS['ring']],
            metacarpals=self.finger_coords_array_np[OCULUS_JOINTS['metacarpals']],
        )

        

    def position_to_array(self, position):
        return np.array([position.x, position.y, position.z])

    def orientation_to_array(self, orientation):
        return np.array([orientation.x, orientation.y, orientation.z, orientation.w])

    def calculate_finger_angles(self, finger_type, finger_joint_coords, metacarpals_coords):
        if finger_type == 'index':
            idx = 1
        elif finger_type == 'middle':
            idx = 2
        elif finger_type == 'ring':
            idx = 3
        elif finger_type == 'little':
            idx = 4
        angle = calculate_angle(
            metacarpals_coords[idx],
            finger_joint_coords[1],
            finger_joint_coords[3]
        )

        return angle

    def calculate_joint_1_angle(self, thumb_joint_coords):

        origin = thumb_joint_coords[1]
        reference_point = thumb_joint_coords[1].copy()
        reference_point[2] += 1

        vector_origin_to_index = reference_point - origin
        vector_origin_to_thumb = thumb_joint_coords[2] - origin

        if np.linalg.norm(vector_origin_to_index) == 0 or np.linalg.norm(vector_origin_to_thumb) == 0:
            print("One of the vectors is zero, unable to compute angle.")
            return np.nan

        z_axis = np.cross(vector_origin_to_index, vector_origin_to_thumb)
        if np.linalg.norm(z_axis) == 0:
            print("Cross product resulted in zero vector; vectors might be parallel.")
            return np.nan

        z_axis /= np.linalg.norm(z_axis)
        x_axis = vector_origin_to_index / np.linalg.norm(vector_origin_to_index)
        y_axis = np.cross(z_axis, x_axis)

        rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

        vector_in_plane = np.dot(rotation_matrix.T, vector_origin_to_thumb)

        angle = np.arctan2(vector_in_plane[1], vector_in_plane[0])

        return angle

    def calculate_joint_2_angle(self, thumb_joint_coords):

        origin = thumb_joint_coords[2]
        vector_origin_to_joint_2 = thumb_joint_coords[3] - origin
        vector_origin_to_joint_0 = thumb_joint_coords[1] - origin

        z_axis = np.cross(vector_origin_to_joint_2, vector_origin_to_joint_0)
        z_axis /= np.linalg.norm(z_axis)

        x_axis = vector_origin_to_joint_2 / np.linalg.norm(vector_origin_to_joint_2)
        y_axis = np.cross(z_axis, x_axis)

        rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

        vector_in_plane = np.dot(rotation_matrix.T, vector_origin_to_joint_0)
        angle = np.arctan2(vector_in_plane[1], vector_in_plane[0])

        return 3.14-angle

    def calculate_joint_3_angle(self, thumb_joint_coords):
        origin = thumb_joint_coords[2]

        vector_origin_to_tip = thumb_joint_coords[3] - origin
        vector_origin_to_joint = thumb_joint_coords[1] - origin

        z_axis = np.cross(vector_origin_to_tip, vector_origin_to_joint)
        z_axis /= np.linalg.norm(z_axis)

        x_axis = vector_origin_to_tip / np.linalg.norm(vector_origin_to_tip)
        y_axis = np.cross(z_axis, x_axis)

        rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

        vector_in_plane = np.dot(rotation_matrix.T, vector_origin_to_joint)
        angle = 3.14 - np.arctan2(vector_in_plane[1], vector_in_plane[0])
        if angle < 0:
            angle = 0
        return angle
    
    def calculate_joint_total_angle(self, thumb_joint_coords):
        origin = thumb_joint_coords[1]

        vector_origin_to_tip = thumb_joint_coords[3] - origin
        vector_origin_to_joint = thumb_joint_coords[0] - origin

        z_axis = np.cross(vector_origin_to_tip, vector_origin_to_joint)
        z_axis /= np.linalg.norm(z_axis)

        x_axis = vector_origin_to_tip / np.linalg.norm(vector_origin_to_tip)
        y_axis = np.cross(z_axis, x_axis)

        rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

        vector_in_plane = np.dot(rotation_matrix.T, vector_origin_to_joint)
        angle = 3.14 - np.arctan2(vector_in_plane[1], vector_in_plane[0])
        if angle < 0:
            angle = 0
        return angle

    def calculate_thumb_angles(self, thumb_joint_coords):
        # thumb 0
        angle0 = self.calculate_joint_total_angle(thumb_joint_coords)

        # thumb 1
        angle1 = calculate_angle_z(
            [1.0,0.0,0.0],
            [0.0,0.0,0.0],
            thumb_joint_coords[1]
        )
        return angle0, angle1
