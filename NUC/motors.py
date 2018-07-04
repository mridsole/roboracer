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


    def _driver_loop(serialpath, pipe):
        """
        The driver thread program.
        """

        # Open the serial connection
        connection = serial.Serial(serialpath, baudrate=115200)

        # Wait 2.5 seconds for boot.
        time.sleep(2.0)

        # Enable motors.
        connection.write(b'<E>')

        r = 1000.
        v = 0.

        while True:

            # Read the latest motor command.
            while pipe.poll():
                r, v = pipe.recv()

            # Send the motor command.
            motor_cmd = b'<M' + \
                '{:.2f}'.format(r).encode() + b',' + \
                '{:.2f}'.format(v).encode() + b'>'

            connection.write(motor_cmd)

            # Read encoder values (in ms)
            wheeltimes = list(
                map(int, connection.readline()[:-1].split(b','))
            )

            # Send the velocity data.
            # TODO: Use Queue instead, so that we can block.
            pipe.send(MotorHAL._velocity_from_times(wheeltimes))


    def __init__(self, serialpath=SERIAL_PATH):

        # Ensure serial path exists.
        if not os.path.exists(serialpath):
            raise Exception('Serial path does not exist: ' + serialpath)


        # Initialize command values.
        self.setpoint_radius = 100.
        self.setpoint_velocity = 1.2

        # Pipe for communicating with the driver process.
        driver_pipe, child_pipe = Pipe()
        self.driver_pipe = driver_pipe
        self.child_pipe = child_pipe

        # Create and start the driver process.
        self.driver_proc = Process(target=MotorHAL._driver_loop, args=(serialpath, child_pipe, ))
        self.driver_proc.start()

        # Initialize velocity state.
        self.vel = (0., 0.)


    def set_cmd(self, r, v):

        self.driver_pipe.send((r, v))


    def get_vel(self):

        # Flush the driver_pipe to get the latest velocity reading.
        # This should be done periodically.
        while self.driver_pipe.poll():
            self.vels = self.driver_pipe.recv()

        return self.vels

