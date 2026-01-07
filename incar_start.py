import asyncio
import socket
import time
import traceback

from incar.webrtc import WebRTCConnection
from incar.messages.robot_command_pb2 import RobotCommand

from inspire_interface import openSerial, write6

ROBOT_COMMAND_CHANNEL = "robot_command"
ROBOT_STATE_CHANNEL = "robot_state"

CONTROL_TIMESTEP = 0.1
CONTROL_TIMEOUT_SECONDS = 0.3
STATE_PUBLISHING_TIMESTEP = 0.1

class IncarHand:
    def __init__(self):
        self.serial = openSerial('/dev/ttyUSB0', 115200)

        self.command = None
        self.last_command_time = time.time() - 10
        
        self._rtc = (WebRTCConnection()
            .add_channel(ROBOT_COMMAND_CHANNEL, lambda msg: self.handle_command_message(msg))
            .add_channel(ROBOT_STATE_CHANNEL)
        )

    async def start(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
    
        future = asyncio.wait(
            [
                asyncio.create_task(self._rtc.start_connection(ip, 9999, True)),
                asyncio.create_task(self._control_loop())
                # TODO: self._state_publishing_loop()
            ], 
            return_when=asyncio.FIRST_EXCEPTION
        )

        try:
            done, _pending = await future
            for task in done:
                task.result()  # raises exceptions if any
            for task in _pending:
                task.cancel()
        except Exception:
            for task in _pending:
                task.cancel()
            raise

    def handle_command_message(self, message: str):
        try:
            message_obj = RobotCommand()
            message_obj.ParseFromString(message)
            print(f"receiving message: {message_obj}")

            command = message_obj.commands.get(f"left.commands.hand.inspire")
            if command is not None:
                print(f"with hand command: {command}")
                self.command = [
                    int(command.values[0]),
                    int(command.values[1]),
                    int(command.values[2]),
                    int(command.values[3]),
                    int(command.values[4]),
                    int(command.values[5])
                ]
                self.last_command_time = time.time()
        except:
            print(traceback.print_exc())


    async def _control_loop(self):
        print("Waiting for first command")
        while self.command == None:
            await asyncio.sleep(1)

        print("First command received. Starting control loop")
        while self._rtc.get_peer().connectionState == "connected":
            try:
                await asyncio.sleep(CONTROL_TIMESTEP)
                if time.time() - self.last_command_time > CONTROL_TIMEOUT_SECONDS:
                    continue

                write6(self.serial, 1, 'angleSet', self.command)
            except Exception:
                raise


if __name__ == "__main__":
    # TODO: Argparse for starting local/remote
    hand = IncarHand()
    asyncio.run(hand.start())