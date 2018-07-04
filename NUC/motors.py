"""
Interface for the motors.
"""

import numpy as np
import serial
import os
import time
from multiprocessing import Process, Pipe

# The USB port used for the motor Arduino

# Wheel radius in metres.
WHEEL_RADIUS = 0.06


class MotorHAL:


    SERIAL_PATH = '/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0'

    
    def _velocity_from_times(wheeltimes):
        # TODO
        return wheeltimes


    def _driver_loop(connection, pipe):
        """
        The driver thread program.
        """

        # Open the serial connection
        self.connection = serial.Serial(serialpath, baudrate=115200)

        # Wait 2.5 seconds for boot.
        time.sleep(2.0)

        # Enable motors.
        connection.write(b'<E>')

        while True:

            # Read the latest motor command.

            # Read encoder values (in ms)
            wheeltimes = list(
                map(int, connection.readline()[:-1].split(b','))
            )

            pipe.send(MotorHAL._velocity_from_times(wheeltimes))


        pass

    def __init__(self, serialpath=SERIAL_PATH):

        # Ensure serial path exists.
        if not os.path.exists(serialpath):
            raise Exception('Serial path does not exist: ' + serialpath)


        # Initialize command values.
        self.setpoint_radius = 100.
        self.setpoint_velocity = 2.

        # Pipe for communicating with the driver process.
        driver_pipe, child_pipe = Pipe()

        self.driver_proc = Process(target=MotorHAL._driver_loop, args=(self.connection, child_pipe))
        self.driver_proc.start()


        # # For now, just loop (TODO: use multiprocess to do async)
        # while True:
        #     
        #     # Formulate the motor command.
        #     motor_cmd = b'<M' + \
        #         '{:.2f}'.format(self.setpoint_radius).encode() + b',' + \
        #         '{:.2f}'.format(self.setpoint_velocity).encode() + b'>'

        #     # Send current motor commmand.
        #     self.connection.write(motor_cmd)

        #     # Read encoder values (in ms)
        #     wheeltimes = list(
        #         map(int, self.connection.readline()[:-1].split(b','))
        #     )
