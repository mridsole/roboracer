"""
The high-level controller that combines trajectory generation and driving.

GOOD VALUES are:

TICKRATE = 20
SLOW_CMD = (0.1, 0.1)
SLOW_TICKS = 3
FAST_TICKS = 5
"""

from multiprocessing import Process, Pipe
import numpy as np
import time

class Controller:

    # Tickrate (Hz) - should be roughly synchronized with the MotorHAL tickrate.
    TICKRATE = 20

    # The slow command to send on such ticks.
    SLOW_CMD = (0.1, 0.1)

    # How many ticks for slow?
    SLOW_TICKS = 3

    # How many ticks for fast?
    FAST_TICKS = 5


    def _next_slow(slowstate):
        is_slow, i = slowstate
        if is_slow:
            if i < Controller.SLOW_TICKS:
                return (is_slow, i + 1)
            else:
                return (not is_slow, 0)
        else:
            if i < Controller.FAST_TICKS:
                return (is_slow, i + 1)
            else:
                return (not is_slow, 0)
    

    def _control_loop(mhal, pipe):

        cmd, direction = ((0,0), 0)
        slow_tick = 0

        slow_state = (False, 0)

        while True:

            # Read the latest trajectory command.
            while pipe.poll():
                cmd, direction = pipe.recv()

            if direction != 0:
                slow_state = Controller._next_slow(slow_state)

            if slow_state[0]:
                cmd = Controller.SLOW_CMD

            mhal.set_cmd(*cmd)

            time.sleep(1/Controller.TICKRATE)


    def __init__(self, mhal):
        
        self.slow_tick = 0
        self.mhal = mhal

        # Make a process for the controller.
        driver_pipe, child_pipe = Pipe()
        self.driver_pipe = driver_pipe
        self.child_pipe = child_pipe

        # Create and start the driver process.
        self.ctrl_proc = Process(
            target=Controller._control_loop, 
            args=(mhal, child_pipe, )
        )
        self.ctrl_proc.start()


    def tick(self, trajectory):
        """
        :returns: The new move.
        """

        # MPC-ish framework: use the provided trajectory to turn etc.
        # TODO: something more complex here?
        # TODO: Implement code for checking if we should stop.

        self.driver_pipe.send(trajectory.immediate_path)


    def set_cmd(self, cmd):
        """
        Manually override the motor command.
        """

        self.driver_pipe.send(cmd)
