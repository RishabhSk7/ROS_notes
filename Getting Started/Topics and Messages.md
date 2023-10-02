
The primary mechanism that ROS nodes use to communicate is to send messages. Messages in ROS are organized into named <b>topics</b>.

The idea is that a node that wants to share information will publish messages on the appropriate topic or topics; a node that wants to receive information will subscribe to the topic or topics that it’s interested in. The ROS master takes care of ensuring that publishers and subscribers can find each other; the messages themselves are sent directly from publisher to subscriber. ^abc0ed

### Communication via topics is many-to-many

^ae9103

Lets assume you run the following command:
```
 rosrun turtlesim turtlesim_node __name:=A 
 rosrun turtlesim turtlesim_node __name:=B 
 rosrun turtlesim turtle_teleop_key __name:=C 
 rosrun turtlesim turtle_teleop_key __name:=D
```
This should start two instances of the turtlesim simulator—These should appear in two separate windows—and two instances of the turtlesim teleoperation node.
Note: ![[Nodes#^938e0c]]
Now try entering direction command in both the teleop terminal.

You might have expected each teleoperation node to connect to one simulator, creating two independently controllable simulations. However, that these two kinds of nodes publish and subscribe, respectively, on the /turtle1/cmd_vel topic. Messages published on this topic, regardless of which node publishes them, are delivered to every subscriber of that topic.

This means that nodes will listen to/publish to topics, regardless of how many publishers/receivers are there. On one end this means a reciever node will listen to a topic regardless of if there are a hundred or a thousand or <b>zero</b> nodes publishing to that topic.  ^3f753a
### Listing topics 
To get a list of active topics, use this command:

` rostopic list`

### Reading messages of a particular topic
You can see the actual messages that are being published on a single topic using the rostopic command: 

`rostopic echo topic-name`

Example:
`rostopic echo /turtle1/cmd_vel`

### Some useful commands

| Command | Description |
|-|-|-|
|rostopic hz topic-name | <br>subscribe the given topic and output statistics in units of messages per second.<br>| ![[Pasted image 20230917235528.png]]|
|rostopic bw topic-name | <br>subscribe the given topic and output statistics in units of bytes per second.<br> | ![[Pasted image 20230917235246.png\|350]]|
|rostopic info topic-name | learn more about a topic. Shows [[#^f0462c\|dataType]] of the topic and other info.| ![[Pasted image 20230917235631.png]]|
|rosmsg show message-type-name | Shows [[#^207325\|details]] of a message (or [[#^55678a\|dataType]]) in a list of fields, one per line.| ![[Pasted image 20230918004118.png]]|


### Viewing the graph
This idea is probably easiest to see graphically, and the easiest way to visualize the publish- subscribe relationships between ROS nodes is to use this command:

`rqt_graph `
^56b9ab

In this name, the r is for ROS, and the qt refers to the Qt GUI toolkit used to implement the program.


#### Graph of Turtlesim
After executing commands to start basic [[Turtlesim Simulator]], you may run `rqt_graph` to observe the communication structure of `turtlesim_node` and `turtle_teleop_key`.
![[Screenshot_20230914_130333.png]]
##### The <b>nodes</b> are represented by ovals, and the topics by rectangles. 

In this case the `/teleop_turtle` is publishing message to a topic called `/turtle1/cmd_vel`. 
`/turtlesim` is subscribed to this topic and receiving messages to execute.


If the <b>Hide: Debug</b> is unchekced we get the following graph:
![[Screenshot_20230914_161920.png]]These are debug topics that are hidden by default. We can make the following observations:
<li>Notice that rqt_graph itself appears as a node.</li>
<li> All of these nodes publish messages on a topic called /rosout, to which the node named /rosout subscribes. This topic is one mechanism through which nodes can generate textual log messages. </li>
<li>The name /rosout refers to both a node and a topic. ROS doesn’t get confused by these kinds of duplicate names because it’s always clear from the context whether we want to talk about the /rosout node or the /rosout topic.</li>[^note]

[^1]:All of these nodes publish messages on a topic called /rosout, to which the node named /rosout subscribes. This topic is one mechanism through which nodes can generate textual log messages.






### Understanding Message Type Names
Every message type belongs to a specific package. Message type names always contain a slash, and the part before the slash is the name of the containing package:
`package-name/type-name`

For example, the `turtlesim/Color` message type breaks down this way:
![[Topics and Messages 2023-09-18 00.29.35.excalidraw]]

This division of message type names serves a few purposes. 
<ul>
<li>Including packages in the message type names helps to prevent name collisions. For example, geometry_msgs/Pose and turtlesim/Pose are distinct message types that contain different (but conceptually similar) data.</li>
<li>When writing ROS programs, we’ll need to declare depen- dencies on other packages that contain message types that we use. Including the package name as part of the message type name makes these dependencies easier to see.</li>
<li>Finally, knowing the package that contains a given message type can be useful for fig- uring out that type’s purpose. For example, the type name ModelState is quite mys- terious in isolation, but the full name gazebo/ModelState clarifies that this message type is part of the Gazebo simulator, and likely contains information about one of the models within that simulation.</li>
</ul>

### Publishing messages from the command line

#### Quick way
`$ rostopic pub -r rate-in-hz topic-name message-type message-content`

This command repeatedly publishes the given message on the given topic at the given rate. The final message content parameter should provide values for all of the fields in the message type, in order.

>It is also possible to read messages from a file (using -f) or from standard input (by omitting both -f and the message content from the command). In both cases, the input should be formatted like the output of rostopic echo.

Here’s an example:
`$ rostopic pub -r 1 /turtle1/cmd_vel geometry_msgs/Twist '[2, 0, 0]' '[0, 0, 0]'`

Note that, `geometry_msgs/Twist` is the Type parameter, and everything after that is data to be passed each time (as a string I guess).

The values are assigned to message fields in the same order that they are shown by `rosmsg show`. In the case, the first three numbers denote the desired linear velocity and the final three numbers denote the desired angular velocity. We use single quotes (’. . . ’) and square brackets ([. . . ]) to group the individual subfields into the two top-level composite fields. 

#### Better Way
<li> The syntax shown above has the distinct disadvantage that you must remember all of the fields of the message type and the order in which they appear. </li>
<li> An alternative is to give single parameter specifying all of the fields as a single YAML (a recursive acronym for “YAML Ain’t Markup Language” ) dictionary.</li>

This command (which does, in fact, contain newline characters) is equivalent to the one above, but it explicitly shows the mapping from field names to values: 

`rostopic pub /turtle1/cmd_vel geometry_msgs/Twist "linear: x: 2.0 y: 0.0 z: 0.0 angular: x: 0.0 y: 0.0 z: 0.0"`

<li> simplest way to get the syntax correct is to use tab completion. Pressing Tab after entering the message type will insert a fully formed YAML dictionary, with all of the fields in the given message type.</li>


### Keynotes
<li>The simulator doesn’t care (or even know) which program publishes those cmd_vel messages. Any program that publishes on that topic can control the turtle.</li>
  <li>The teleoperation program doesn’t care (or even know) which program subscribes to the cmd_vel messages it publishes. Any program that subscribes to that topic is free to respond to those commands.</li>

Footnotes linked in various places in note:
> The word “type” in this context is referring to the concept of a data type. It’s important to understand message types because they determine the content of the messages. That is, the message type of a topic tells you what information is included in each message on that topic, and how that information is organized. For example, `rostopic info /turtle1/color_sensor` returns:
> Type: turtlesim/Color
>Publishers: 
> * /turtlesim (http://Rishabh:44685/)
>
>Subscribers: None

^55678a

>  Each field is defined by a built-in data type (like int8, bool, or string) and a field name. For example, `rosmsg show turtlesim/Color` returns:
>  uint8 r 
>  uint8 g 
>  uint8 b
>The output above tells us that a turtlesim/Color is a thing that contains three unsigned 8-bit integers called r, g, and b.

^207325

