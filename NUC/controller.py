"""
The high-level controller that combines trajectory generation and driving.
"""

class Controller:

    
    # Slow every SLOW_EVERY ticks.
    SLOW_EVERY = 4

    # The slow command to send on such ticks.
    SLOW_CMD = (0.1, 0.1)


    def __init__(self):
        
        self.slow_tick = 0


    def tick(self, trajectory):
        """
        :returns: The new move.
        """

        # MPC-ish framework: use the provided trajectory to turn etc.
        cmd, direction = trajectory.immediate_path

        if direction != 0:
            self.slow_tick += 1

        if self.slow_tick % Controller.SLOW_EVERY:
            cmd = Controller.SLOW_CMD

        return cmd
