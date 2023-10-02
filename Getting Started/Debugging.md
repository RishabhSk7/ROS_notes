One final (for now) command line tool, which can be helpful when ROS is not behaving the way you expect, is `roswtf` which can be run with no arguments: 

`roswtf`

This command performs a broad variety of sanity checks, including examinations of your environment variables, installed files, and running nodes. For example, roswtf checks whether the rosdep portions of the install process have been completed, whether any nodes appear to have hung or died unexpectedly, and whether the active nodes are cor- rectly connected to each other. A complete list of checks performed by roswtf seems to exist, unfortunately, only in the Python source code for that tool.