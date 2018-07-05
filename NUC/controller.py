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

# TODO: Need to stop before we turn.

class Controller:

    # Tickrate (Hz) - should be roughly synchronized with the MotorHAL tickrate.
    TICKRATE = 20

    # Command for stopping.
    # If we get a turn start, we need to stop for some ticks.
    STOP_CMD = (0., 0.)

    # How many ticks to send th stop command for.
    STOP_TICKS = 0

    # The slow command to send on such ticks.
    SLOW_CMD = (0.00, 0.00)

    # How many ticks for slow?
    SLOW_TICKS = 5

    # How many ticks for fast?
    FAST_TICKS = 4


    def _next_slow(slowstate):
        is_init, is_slow, i = slowstate

        # If we ust started turning, we need to stop for a sec.
        if is_init:
            if i < Controller.STOP_TICKS:
                return (True, is_slow, i + 1)
            else:
                return (False, is_slow, 0)
        
        if is_slow:
            if i < Controller.SLOW_TICKS:
                return (False, is_slow, i + 1)
            else:
                return (False, not is_slow, 0)
        else:
            if i < Controller.FAST_TICKS:
                return (False, is_slow, i + 1)
            else:
                return (False, not is_slow, 0)
    

    def _control_loop(mhal, pipe):

        cmd, direction, stop = ((0,0), 0, 0)
        slow_tick = 0

        slow_state = (False, False, 0)

        while True:

            # Read the latest trajectory command.
            while pipe.poll():
                cmd, direction, stop = pipe.recv()

            # if stop:
            #     slow_state = (True, False, 0)
            #     cmd = Controller.STOP_CMD
            #     print('Init stop')
            if direction != 0:
                slow_state = Controller._next_slow(slow_state)
                is_init, is_slow, i = slow_state
                if is_init:
                    cmd = Controller.STOP_CMD
                elif is_slow:
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

        self.cmd_last = None


    def tick(self, trajectory):
        """
        :returns: The new move.
        """

        # MPC-ish framework: use the provided trajectory to turn etc.
        # TODO: something more complex here?
        # TODO: Implement code for checking if we should stop.

        cmd = trajectory.immediate_path

        c, d = cmd
        if self.cmd_last is not None:
            c_last, d_last = self.cmd_last
            if d_last == 0 and d != d_last:
                # TODO: init stop state
                self.driver_pipe.send((c, d, True))
                self.cmd_last = cmd
                print('Sending stop')
                return

        self.cmd_last = cmd
        self.driver_pipe.send((c, d, False))



    def set_cmd(self, cmd):
        """
        Manually override the motor command.
        """

        a, b = cmd
        self.driver_pipe.send((a, b, False))
