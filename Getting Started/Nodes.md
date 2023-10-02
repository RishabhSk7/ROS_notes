Once you’ve started roscore, you can run programs that use ROS. A running instance of a ROS program is called a node.

The phrase “running instance of” in this definition is important. If we execute mul- tiple copies of the same program at the same time—taking care to ensure that each uses a different node name—each of those copies is treated as a separate node.

### Starting nodes 
The basic command to create a node (also known as “running a ROS pro- gram”) is rosrun:
```
rosrun package-name executable-name
```

`rosrun` is nothing fancy, just a script that takes argument of a package and which executable in package to run. It just knows, or takes a guess, at where the executable should be and runs it.

Further, the ` __name:=A` argument will give the node a  custom name "A". This lets us run mulitple instance of a node, which roscore would otherwise restrict. ^938e0c

### Listing nodes
ROS provides a few ways to get information about the nodes that are running at any particular time. To get a list of running nodes, try this command(requires master to be running):
```
rosnode list
```


>The /rosout node is a special node that is started automatically by roscore. Its pur- pose is somewhat similar to the standard output (i.e. std::cout) that you might use in a console program. 

> The leading / in the name /rosout indicates that this node’s name is in the global namespace. ROS has a rich system for naming nodes and other ob- jects. This system, in later notes, uses name- spaces to organize things.

### Custon node names
If you compare the output of rosnode list to the executable names in the rosrun commands from Section 2.3, you’ll notice that node names are not necessarily the same as the names of the executables underlying those nodes.  You can explicitly set the name of a node as part of the rosrun command: 

```
rosrun package-name executable-name __name:=node-name
```

This approach will override the name that the node would normally have given itself, and can be important because ROS insists that every node have a distinct name. 

### Inspecting a node 
You can get some information about a particular node using this command: 

```
rosnode info node-name
```

### Killing a node 
To kill a node you can use this command: 
```
rosnode kill node-name
```


You can also kill a node using the usual Ctrl-C technique. However, that method may not give the node a chance to unregister itself from the master. A symptom of this problem is that the killed node may still be listed by rosnode list for a while. This is harmless, but might make it more difficult to tell what’s going on. To remove dead nodes from the list, you can use this command: 
```
rosnode cleanup
```

### Nodes are loosely coupled
This means that nodes are not aware of each other existence. This will coincide, and further explained in the topic [[Topics and Messages#^ae9103|Communication of nodes is many to many]]. Ultimately, ![[Topics and Messages#^3f753a]]
Read up on Topics and Messages next.