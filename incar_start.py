import traceback

from inspire_interface import openSerial, write6, read6
from incar_networking.robot_interface import IncarRobotInterface

class InspireHand:
    def __init__(self):
        self.serial = openSerial('/dev/ttyUSB0', 115200)

    def move_joints(self, joint_positions: list[float]):
        try:
            command = [
                int(joint_positions[0]),
                int(joint_positions[1]),
                int(joint_positions[2]),
                int(joint_positions[3]),
                int(joint_positions[4]),
                int(joint_positions[5])
            ]
            write6(self.serial, 1, 'angleSet', command)
        except:
            traceback.print_exc()
    
    def publish_state(self, interface: IncarRobotInterface):
        try:
            currentAngles = read6(self.serial, 1, 'angleAct')
            currentForces = read6(self.serial, 1, 'forceAct')
            interface.set_robot_state("hand", joint_pos=currentAngles, joint_effort=currentForces)
            interface.publish_state()
        except:
            traceback.print_exc()

if __name__ == "__main__":
    robot = InspireHand()
    interface = IncarRobotInterface(
        0.1,
        command_hooks = {
            "right.commands.hand.inspire": robot.move_joints
        },
        loop_callbacks = [
            robot.publish_state
        ]
    )
    interface.start("127.0.0.1")