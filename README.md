# motion_controller
Given desired twists in a controlled frame (eg odometric or global) relative to the rover, determine the way the rover will convert them to controller commands. Think like adding cost function on how the rover can move in situ (eg. limit diagonal movements in a meccanum wheel etc)
