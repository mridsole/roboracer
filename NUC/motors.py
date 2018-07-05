"""
Interface for the motors.
"""

import numpy as np
import serial
import os
import time
from multiprocessing import Process, Pipe

# TODO: need to change this to use (vl, vr) instead of (r, v)

# Wheel radius in metres.
WHEEL_RADIUS = 0.06


class MotorHAL:


    SERIAL_PATH = '/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0'

    
    def _velocity_from_times(wheeltimes):

        # TODO: Leave for now, encoders fucked

        # Use the wheel radius to compute the velocity

        # Average the left wheels and the right
        FL, FR, BL, BR = wheeltimes

        # Special cases: handle bad reports
        if FL == 0 and BL != 0:
            FL = BL
        elif FL != 0 and BL == 0:
            BL = FL

        if FR == 0 and BR != 0:
            FR = BR
        elif FR != 0 and BR == 0:
            BR = FR


        left = (FL + BL) / 2
        right = (FR + BR) / 2

        left_lin = 0
        if left != 0:
            left_lin = 2*np.pi*(1 / left)

        right_lin = 0
        if right != 0:
            right_lin = 2*np.pi*(1 / right)

        return (left_lin, right_lin)


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

        vl = 0.
        vr = 0.

        while True:

            # Read the latest motor command.
            while pipe.poll():
                vl, vr = pipe.recv()

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

        # Pipe for communicating with the driver process.
        driver_pipe, child_pipe = Pipe()
        self.driver_pipe = driver_pipe
        self.child_pipe = child_pipe

        # Create and start the driver process.
        self.driver_proc = Process(target=MotorHAL._driver_loop, args=(serialpath, child_pipe, ))
        self.driver_proc.start()

        # Initialize velocity state.
        self.vel = (0., 0.)


    # def set_cmd(self, r, v):

    #     self.driver_pipe.send((r, v))


    def set_cmd(self, vl, vr):

        self.driver_pipe.send((vl, vr))


    def get_vel(self):

        # Flush the driver_pipe to get the latest velocity reading.
        # This should be done periodically.
        while self.driver_pipe.poll():
            self.vels = self.driver_pipe.recv()

        return self.vels

