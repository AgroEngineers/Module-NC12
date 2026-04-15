from enum import Enum
from pathlib import Path
from typing import Union, Dict

import cv2
import numpy
import serial
import serial.tools.list_ports
from asm.api.base import ContainerParameterResults, \
    ModuleInformation, ModuleConfiguration, ModuleConfigurationPattern
from asm.api.hardware import ASMHardware, AvailableDevices


class GateStates(Enum):
    OPEN = 0
    LEFT = 45
    RIGHT = 135


class Direction(Enum):
    STOP = 0
    FORWARD = 1
    BACKWARD = 2


class Nc12(ASMHardware):
    NAME: str = 'NC12'
    VERSION: str = '2.0'
    BAUD_RATE: str = '115200'

    CONFIGURATION: Dict[str, ModuleConfigurationPattern]

    ACTIVE_CAMERA: cv2.VideoCapture
    ACTIVE_MACHINE: serial.Serial

    CURRENT_DIRECTION: Direction = Direction.STOP
    CURRENT_STATES: dict[int, str] = {}

    def configuration(self, configuration: ModuleConfiguration):
        self.CONFIGURATION = configuration.configuration

    def get_available_devices(self) -> AvailableDevices:
        ports: list[str] = []
        cameras: list[str] = []

        for port in serial.tools.list_ports.comports():
            if "USB" in port.device:
                port.append(port.device)

        for path in Path("/dev/").glob("video*"):
            cameras.append(str(path.absolute()))

        return AvailableDevices(ports, cameras)

    def connect_camera(self, port: str) -> bool:
        self.ACTIVE_CAMERA = cv2.VideoCapture(int(port[10:]))

        if not self.ACTIVE_CAMERA.isOpened():
            return False
        return True

    def connect_machine(self, port: str) -> bool:
        self.ACTIVE_MACHINE = serial.Serial(port, self.BAUD_RATE)

        if not self.ACTIVE_MACHINE.is_open:
            return False

        for gate in self.CONFIGURATION["servos"]:
            self.CURRENT_STATES[gate] = GateStates.OPEN.name

        self.ACTIVE_MACHINE.write(str({
            "servos": self.CONFIGURATION["servos"],
            "motors": self.CONFIGURATION["motors"]
        }))

        return True

    def disconnect_camera(self) -> None:
        self.ACTIVE_CAMERA.release()

    def disconnect_machine(self) -> None:
        self.set_container(len(self.CONFIGURATION["containers"]))
        self.set_direction(Direction.STOP.name)

        self.ACTIVE_MACHINE.close()

    def frame(self) -> Union[numpy.ndarray, None]:
        success, frame = self.ACTIVE_CAMERA.read()
        if not success:
            return None

        return frame

    def set_direction(self, direction: str) -> None:
        self.ACTIVE_MACHINE.write(str({"direction": Direction[direction].value}))

    def set_container(self, container: int) -> None:
        current_container = self.CONFIGURATION["containers"][container]

        for i in current_container:
            self.set_gate(i.name, i.value)

    def get_available_gates(self) -> int:
        return len(self.CONFIGURATION["servos"])

    def get_available_gate_states(self) -> list[str]:
        return [state.name for state in GateStates]

    def get_current_states(self) -> dict[int, str]:
        return self.CURRENT_STATES

    def set_gate(self, gate: int, state: str) -> None:
        self.ACTIVE_MACHINE.write(str({"gate": gate, "angle": GateStates[state].value, "inverted": bool(self.CONFIGURATION["servos"][gate]["inverted"])}))
        self.CURRENT_STATES[gate] = state

    def canvas(self) -> str:
        pass

    def process(self) -> Union[list[ContainerParameterResults], None]:
        return None

    def module_info(self) -> ModuleInformation:
        return ModuleInformation(
            self.NAME,
            self.VERSION,
            [],
            ModuleConfiguration({
                "motors": [
                    [5, 4],
                    [3, 2]
                ],
                "servos": [
                    {
                        "port": 1,
                        "inverted": False
                    },
                    {
                        "port": 2,
                        "inverted": True
                    },
                    {
                        "port": 3,
                        "inverted": False
                    }
                ],
                "containers": [
                    {
                        "1": "LEFT"
                    },
                    {
                        "1": "RIGHT"
                    },
                    {
                        "2": "RIGHT"
                    },
                    {
                        "2": "LEFT"
                    },
                    {
                        "3": "LEFT"
                    },
                    {
                        "3": "RIGHT"
                    },
                    {}
                ]
            }),
            [],
            []
        )
