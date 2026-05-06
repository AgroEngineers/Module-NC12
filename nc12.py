import asyncio
import json
import time
from enum import Enum
from pathlib import Path
from typing import Union, Dict, Optional
import threading

import cv2
import numpy
import serial
import serial.tools.list_ports
from asm.api.base import ContainerParameterResults, \
    ModuleInformation, ModuleConfiguration, ModuleConfigurationPattern, ModuleTask, ModuleTaskInput, ModuleTaskOutput
from asm.api.hardware import ASMHardware, AvailableDevices
from asm import logman

class GateStates(Enum):
    OPEN = "open"
    LEFT = "left"
    RIGHT = "right"


class Direction(Enum):
    STOP = 0
    FORWARD = 1
    BACKWARD = 2


class Nc12(ASMHardware):
    NAME: str = 'NC12'
    VERSION: str = '2.0'
    BAUD_RATE: int = 115200

    CONFIGURATION: Dict[str, ModuleConfigurationPattern]

    ACTIVE_CAMERA: Optional[cv2.VideoCapture] = None
    ACTIVE_MACHINE: Optional[serial.Serial] = None

    CURRENT_DIRECTION: Direction = Direction.STOP
    CURRENT_STATES: dict[int, str] = {}

    _lock = threading.Lock()

    def configuration(self, configuration: ModuleConfiguration):
        self.CONFIGURATION = configuration.configuration

    def get_available_devices(self) -> AvailableDevices:
        ports: list[str] = []
        cameras: list[str] = []

        for path in Path("/dev/").glob("ttyUSB*"):
            ports.append(str(path.absolute()))

        for path in Path("/dev/").glob("video*"):
            cameras.append(str(path.absolute()))

        return AvailableDevices(ports, cameras)

    def is_camera_connected(self) -> bool:
        return self.ACTIVE_CAMERA is not None and self.ACTIVE_CAMERA.isOpened()

    def is_machine_connected(self) -> bool:
        return self.ACTIVE_MACHINE is not None and bool(self.ACTIVE_MACHINE.is_open)

    async def connect_camera(self, port: str) -> bool:
        self.ACTIVE_CAMERA = cv2.VideoCapture(int(port[10:]))

        if not self.ACTIVE_CAMERA.isOpened():
            return False
        return True

    def read_serial_thread(self, ser_obj):
        while ser_obj.is_open:
            try:
                if ser_obj.in_waiting > 0:
                    line = ser_obj.readline().decode('utf-8', errors='replace').strip()
                    if line:
                        logman.log(f"Data from serial: {line}")
            except Exception as e:
                logman.log(f"Serial Error: {e}")
                break

    async def connect_machine(self, port: str) -> bool:
        print(port)
        self.ACTIVE_MACHINE = serial.Serial(port, self.BAUD_RATE)
        
        thread = threading.Thread(target=self.read_serial_thread, args=(self.ACTIVE_MACHINE,), daemon=True)
        thread.start()

        if not self.ACTIVE_MACHINE.is_open:
            return False

        for gate in range(len(self.CONFIGURATION["servos"])):
            self.CURRENT_STATES.update({gate: GateStates.OPEN.value})
        
        time.sleep(2)
        # Sorry for hardcode, I haven't time to fix it
        # TODO: Remove hardcode
        
        self.ACTIVE_MACHINE.write((json.dumps({
            "task": "motors",
            "motors":[[7,4,5],[3,2,6]]
        }) + "\n").encode('utf-8'))
        time.sleep(0.2)
        self.ACTIVE_MACHINE.write((json.dumps({
            "task": "servos",
            "servos": [1, 2, 3]
        }) + "\n").encode('utf-8'))

        for servo in self.CONFIGURATION["servos"]:
            self.set_gate(servo["port"]-1, GateStates.OPEN.value)
            time.sleep(1.5)

        return True

    async def disconnect_camera(self):
        with self._lock:
            cap = self.ACTIVE_CAMERA
            self.ACTIVE_CAMERA = None

        if cap:
            cap.release()

    async def disconnect_machine(self) -> None:
        if not self.is_machine_connected():
            return
        self.set_container(len(self.CONFIGURATION["containers"]))
        self.set_direction(Direction.STOP.name)

        self.ACTIVE_MACHINE.close()

    def frame(self):
        with self._lock:
            if self.ACTIVE_CAMERA is None:
                return None
            if not self.ACTIVE_CAMERA.isOpened():
                return None

            success, frame = self.ACTIVE_CAMERA.read()
            if not success:
                return None
            return frame

    def set_direction(self, direction: str) -> None:
        with self._lock:
            self.ACTIVE_MACHINE.write(
                json.dumps({"task": "direction", "direction": Direction[direction.upper()].value}).encode('utf-8'))

    
    def set_container(self, container: int) -> dict:
        current_container = self.CONFIGURATION["containers"][container - 1]

        ress = {
            "type": "sync",
            "motor": self.get_forward_direction().lower()
        }

        self.set_direction(self.get_forward_direction())

        for i in range(len(self.CONFIGURATION["servos"])):
            self.set_gate(i, "open")
            time.sleep(0.05)

        for gate, state in current_container.items():
            self.set_gate(int(gate)-1, state.lower())
            ress.update({f"servo_{gate}": state.lower()})

        return ress

    def get_available_gates(self) -> int:
        return len(self.CONFIGURATION["servos"])

    def get_available_gate_states(self) -> list[str]:
        return [state.name for state in GateStates]

    def get_current_states(self) -> dict[int, str]:
        return self.CURRENT_STATES

    def _get_angle_by_state(self, gate: int, state: str) -> int:
        for servo in self.CONFIGURATION["servos"]:
            if servo.get("port") == gate+1:
                if state == "None" or state is None:
                    return servo["states"]["open"] 
                if "states" not in servo:
                    raise ValueError(f"Servo {gate} has no 'states': {servo}")
                if state not in servo["states"]:
                    raise ValueError(f"Invalid state '{state}' for gate {gate}")

                return servo["states"][state]

        raise ValueError(f"Gate {gate} not found")

    def set_gate(self, gate: int, state: str) -> None:
        logman.log(f"Gate: {gate}, state: {state}")
        
        self.ACTIVE_MACHINE.write(
            (json.dumps({
                "task": "gate",
                "gate": gate,
                "angle": self._get_angle_by_state(gate, state)
            }) + "\n").encode('utf-8')
        )

        self.CURRENT_STATES[gate] = state

    def get_container_count(self) -> int:
        return len(self.CONFIGURATION["containers"])

    def get_forward_direction(self) -> str:
        return Direction.FORWARD.name

    def canvas(self) -> str:
        pass

    def process(self) -> Union[list[ContainerParameterResults], None]:
        return None

    def task(self, task: ModuleTask, task_input: ModuleTaskInput) -> Union[ModuleTaskOutput, None]:
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
                        "states": {
                            "open": 180,
                            "left": 135,
                            "right": 45
                        }
                    },
                    {
                        "port": 2,
                        "states": {
                            "open": 0,
                            "left": 45,
                            "right": 135
                        }
                    },
                    {
                        "port": 3,
                        "states": {
                            "open": 180,
                            "left": 135,
                            "right": 45
                        }
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
            })
        )
