So far we’ve talked primarily about files, and how they are organized into packages. Let’s shift gears and talk now about how to actually execute some ROS software. One of the basic goals of ROS is to enable roboticists to design software as a collection of small, mostly independent programs called nodes that all run at the same time. For this to work, those nodes must be able to communicate with one another. The part of ROS that facilitates this communication is called the ROS master. To start the master, use this command: 
```
roscore
```

You should allow the master to continue running for the entire time that you’re using ROS. One reasonable workflow is to start roscore in one terminal, then open other termi- nals for your “real” work. There are not many reasons to stop roscore, except when you’ve finished working with ROS. When you reach that point, you can stop the master by typing Ctrl-C in its terminal.

NOTE: Most ROS nodes connect to the master when they start up, and do not attempt to reconnect if that connection fails later on. Therefore, if you stop roscore , any other nodes running at the time will be unable to establish new connections, even if you restart roscore later.