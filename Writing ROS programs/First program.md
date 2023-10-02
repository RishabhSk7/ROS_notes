[[Setting up Workspace]]
We are using C++ in this case.

### Creating the C++ file
in your package directory, right next to `package.xml` and `CMakeLists.txt`, make a file called `hello.cpp`.

![[Setting up Workspace#^33214c]]

<b>Ignore if there are any errors reported while writing the code, I'll try and fix them as we go.</b>

open `hello.cpp` in a text editor, and add the following lines:
```Cpp
#include <ros/ros.h>

int main(int argc, char ** argv) {
	ros::init(argc, argv, "hello_ros");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("Hello ROS");
}

```

`#include <ros/ros.h>`
	Includes the header files necessary for executing ros programs. You'll want to include this header files everywhere in your program.

`int main(int argc, char ** argv) {`
	Starts the main function, takes any command line args.

`ros::init(argc, argv, "hello_ros");`
	Initialises the ROS client library. Called once during the beginning of the program. The last argument is the default name of the node.

`ros::NodeHandle nh;`
	The ros::NodeHandle object is the main mechanism that your program will use to interact with the ROS system. Creating this object registers your program as a node with the ROS master. The simplest technique is to create a single NodeHandle object to use throughout your program.

> 	Internally, the NodeHandle class maintains a reference count, and only regis- ters a new node with the master when the first NodeHandle object is created. Likewise, the node is only unregistered when all of the NodeHandle objects have been destroyed. This detail has two impacts: First, you can, if you pre- fer, create multiple NodeHandle objects, all of which refer to the same node. There are occasionally reasons that this would make sense. An example of one such situation appears on page 129. Second, this means that it is not possible, using the standard roscpp interface, to run multiple distinct nodes within a single program.

`ROS_INFO_STREAM("Hello ROS");` 
	This line generates an informational message. This mes- sage is sent to several different locations, including the console screen.

### Compiling the Program
This is handled by the ROS build system called 'catkin', in four steps:

#### Declaring Dependencies
First, we need to declare the other packages on which our package depends. For C++ programs, this step is needed primarily to ensure that catkin provides the C++ compiler with the appropriate flags to locate the header files and libraries that it needs.

Dependencies are declared in the `CMakeLists.txt` as was created  in the package.

The default version of this file has this line: 
	 `find_package(catkin REQUIRED)`

Dependencies on other catkin packages can be added in a COMPONENTS section in this format:
	`find_package(catkin REQUIRED COMPONENTS package-names)`
	
<b>I hope I don't need to provide template for each command, and you've got more than *2* brain cells.</b>

For this our hello.cpp, we only need the C++ ROS client library. This can be declared by:
>	`find_package(catkin REQUIRED COMPONENTS roscpp)`

We should also list these dependencies in the `package.xml`. This is not necessary for us, but will be for others when they build it. 
The code will be:
>	`<build_depend>roscpp</build_depend>`
>	`<run_depend>roscpp</run_depend>`



#### Declaring an executable
Now that we've declared all the dependencies, we'll need need to declare the files that need to be executed when the "node" is run.

Next add the line following line to `CMakeLists.txt`:
	`add_executable(hello hello.cpp)`
	The First is the name of the executable as will be called from the terminal. The second is the name of the source file we will be combined to form the executable. If there's more than one source file, add the source files with a space. 

Now add another line:
	`target_link_libraries(hello ${catkin_LIBRARIES})`
	This line makes catkin link the packages we mentioned previously (roscpp in this case) to our executable.

If you have more than one executable, modify the above 2 line for each one as neccesary.

#### Building the workspace

Go to the root of your [[Setting up Workspace#^1fb4ec|workspace]]. The one where [[Setting up Workspace#^c0e22f|your src folder is located]].

Open terminal in this directory and run:
	`catkin_make`

>If you see errors from catkin_make that the header ros/ros.h cannot be found, or “undefined reference” errors on ros::init or other ROS functions, the most likely reason is that your CMakeLists.txt does not correctly declare a dependency on roscpp.

#### Sourcing setup.bash

^75fc90

The final step is to execute a script called setup.bash, which is created by catkin_make inside the devel subdirectory of your workspace. Go to your workspace folder and run in terminal: 
	`source devel/setup.bash`

This automatically-generated script sets several environment variables that enable ROS to find your package and its newly-generated executables.

Ideally this needs to be run each time you open terminal, but you can automate this by opening `.bashrc` in root directory and adding the following line: 
	`source {PATH_TO_WORKSPACE}/devel/setup.bash`

Remember to substitute in your actual workspace path. You'll add the location to you environment variables if you're working on windows ig.

#### Running the program

I would like to thank the God(s) and any and all higher powers that existed and made me power through this process, my friends for supporting me and myself for being a genius. 

Start roscore by running in a terminal:
	`roscore`

Then in another terminal run:
	`rosrun test hello`

`test` is the name for [[Setting up Workspace#^a88477|my package]].

#### Output

It should look something like:
	`\[ INFO] \[1416432122.659693753]: Hello ROS`

BTW, the string of random numbers after "INFO" is time since EPOCH time to when out line was executed.

<b>If you get any errors</b>, you're either misspelling the package name, or incorrectly or not executing [[First program#^75fc90|setup.bash]]. Remember it needs to be executed in each terminal instance, or should be edited into the `~/.bashrc` file.

[[Publisher Program]]
