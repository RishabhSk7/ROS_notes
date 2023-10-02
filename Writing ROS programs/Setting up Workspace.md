In case you've not noticed, I have not bothered with installation help as it was rather bothersome and unique for me. Btw I use arch, 
and it was/is pretty confusing to set up. The official docs have old instructions and often result in errors. Ultimately, I ended up  on a repo called arch4edu that hade pre complied binaries for me. If you're use windows I think the route should be a bit more easy for you, but for installation, you're on your own.  ^9d36f4

### Creating Workspace and package
All ROS software, including software we create, is organized into packages. Before we write any programs, the first steps are to create a workspace to hold our packages, and then to create the package itself.

Workspace is basically the main folder you'll work in. ^1fb4ec

<b>Create another folder INSIDE the workspace called `src`. This will have source code for your packages.</b> ^c0e22f

### Creating a package
The command to create a new ROS package, <b>which should be run from the src directory of your workspace</b>, looks like this:

`catkin_create_pkg package-name`

I called my package "test". ^a88477

Actually, this package creation command doesn’t do much: It creates a directory to hold the package and creates two configuration files inside that directory.

<li>The first configuration file, called package.xml, is the manifest file. This file defines some details about the package, including its name, version, maintainer, and de- pendencies. Any directory with package.xml is called a package.</li>
<li>Another file called CMakeLists.txt, is a script for a cross- platform build system called CMake. It contains a list of build instructions including what executables should be created, what source files to use to build each of them, and where to find the include files and libraries needed for those executables. CMake is used internally by catkin. </li>

### To find  a package
`rospack find package-name`

Ofc you dont remember all package names. Remember tab completion?

### Editing the manifest 
After creating your package, you may want to edit its package.xml, which contains some metadata describing the package. The default version installed by catkin_create_pkg is liberally commented and largely self-explanatory. Note, however, that most of this information is not utilized by ROS, neither at build time nor at run time, and only becomes really important if you release your package publicly. In the spirit of keeping documentation in sync with actual functionality, a reasonable minimum might be to fill in the description and maintainer fields.

>Some online tutorials suggest creating a src directory within your package directory to contain C++ source files. This additional organization might be helpful, espe- cially for larger packages with many types of files, but it isn’t strictly necessary.

^33214c

[[First program]]