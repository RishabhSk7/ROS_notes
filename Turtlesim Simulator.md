<h3>Starting turtlesim</h3>
In three separate terminals, execute these three commands:
```
roscore
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key
```


`roscore` is the [[The Master]].
`rosrun` command runs the the indivisual [[Nodes]]. The next arguments are package name and executable name respectively. Read more here: [[Nodes#Starting nodes]].

`turtlesim_node` is the executable that starts the turle gui, and listen to [[Topics and Messages#^abc0ed|topics]] to get input on how to move the turtle.

`turtle_teleop_key` is the executable that when runs takes input from the keyboard and passes it to a topic which the `turtleism_mode` is listening to.

> If your 'turtle' is not moving, make sure the terminal with `turtle_teleop_key`  has focus.

### Refer below to working of the two executables:
![[Topics and Messages#Graph of Turtlesim]]



