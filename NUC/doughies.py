"""
The final real-time control loop.
"""

import numpy as np
import curses
from curses import wrapper
from motors import MotorHAL

# First initialize the motor HAL which opens a connection with the Arduino.
# This will make the wheels spin for a couple of seconds.

# mhal = MotorHAL()
# controller = Controller(mhal)

stdscr = curses.initscr()
curses.noecho()
curses.cbreak()

def main(stdscr):

    while True:
        k = stdscr.getkey()

        # Commands manually

        if k == 's' or k == 'S':
            mhal.set_cmd((0., 0.))

        if k == 'w':
            mhal.set_cmd((0.3, 0.3))

        if k == 'W':
            mhal.set_cmd((1.1, 1.1))

        if k == 'a':
            mhal.set_cmd((0.2, 0.8))

        if k == 'A':
            mhal.set_cmd((0.15, 1.4))

        if k == 'd':
            mhal.set_cmd((0.8, 0.2))

        if k == 'D':
            mhal.set_cmd((1.4, 0.15))


wrapper(main)

# while True:
# 
#     # This isn't actually the "tick", this is setting the move reference based
#     # on state.
#     # cmd = controller.tick(traj)
#     # print(traj.immediate_path[1])
# 
#     # Set motor command. TODO: deprecate, this is handled by the controller.
#     # mhal.set_cmd(*cmd)
# 
#     # Poll for velocity (why not?)
#     stdscr.refresh()
#     k = stdscr.getkey()
#     # print(k)
