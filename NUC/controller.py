"""
The high-level controller that combines trajectory generation and driving.
"""

from multiprocessing import Process, Pipe
import numpy as np

class Controller:

    
    # Slow every SLOW_EVERY ticks.
    SLOW_EVERY = 2

    # The slow command to send on such ticks.
    SLOW_CMD = (0.15, 0.15)

    
    def _control_loop(self):


    def __init__(self):
        
        self.slow_tick = 0

        # Make a process for the controller.


    def tick(self, trajectory):
        """
        :returns: The new move.
        """

        # MPC-ish framework: use the provided trajectory to turn etc.
        cmd, direction = trajectory.immediate_path

        if direction != 0:
            self.slow_tick += 1

        if self.slow_tick % Controller.SLOW_EVERY == 0:
            cmd = Controller.SLOW_CMD

        return cmd
