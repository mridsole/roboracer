"""
The high-level controller that combines trajectory generation and driving.
"""

from multiprocessing import Process, Pipe
import numpy as np

class Controller:

    # Tickrate (Hz)
    TICKRATE = 6
    
    # Slow every SLOW_EVERY ticks.
    SLOW_EVERY = 2

    # The slow command to send on such ticks.
    SLOW_CMD = (0.15, 0.15)

    
    def _control_loop(mhal, pipe):

        cmd, direction = ((0,0), 0)
        slow_tick = 0

        while True:

            # Read the latest trajectory command.
            while pipe.poll():
                cmd, direction = pipe.recv()

            if direction != 0:
                slow_tick += 1

            if slow_tick % Controller.SLOW_EVERY == 0:
                cmd = Controller.SLOW_CMD


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
        self.driver_pipe.send(trajectory.immediate_path)

    def set_cmd(self, cmd)
        """
        Manually override the motor command.
        """

        self.driver_pipe.send(cmd)
