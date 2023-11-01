## **Getting Started**
### **Installing matlab**
This repository uses pip to install matlab in the docker environment. To install pip and matlab, follow the below instructions.
```bash
$ apt update
$ apt install python3-pip
$ pip install matplotlib
```

### **Cloning this Repository**
Using the cs225_RRT_project repository requires desktop [configuration of a GitHub SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account).

Clone this repository in your desired location by running the following command in a terminal:
```bash
# Clone the repository
$ git clone git@github.com:RiceMiceA/cs225_RRT_project.git
```
### 

## **Preparing Your Code**
We're using cmake instead of make like the MPs. This allows for us to use libraries such as Catch2 that can be installed in your system rather than providing them with each assignment. This change does mean that for each assignment you need to use CMake to build your own custom makefiles. To do this you need to run the following in the base directory of the assignment. Which in this assignment is the mp_mazes directory.
```bash
$ mkdir build
$ cd build
```
This first makes a new directory in your assignment directory called build. This is where you will actually build the assignment and then moves to that directory. This is not included in the provided code since we are following industry standard practices and you would normally exclude the build directory from any source control system.

Now you need to actually run CMake as follows.
```bash
$ cmake ..
```
This runs CMake to initialize the current directory which is the build directory you just made as the location to build this repository.

## **Quick Run**
We're planning on perfecting an sh file to quickly run the repo by using this instruction:
```bash
$ ./run.sh
```

## References
<a id="1">[1]</a>
LaValle, Steven M.. “Rapidly-exploring random trees : a new tool for path planning.” The annual research report (1998): n. pag.

<a id="2">[2]</a>
(No date a) Robotic Motion Planning: RRT’s - CMU school of computer science. Available at: https://www.cs.cmu.edu/~motionplanning/lecture/lec20.pdf (Accessed: 01 November 2023). 

<a id="3">[3]</a>
Rapidly exploring random tree (2023) Wikipedia. Available at: https://en.wikipedia.org/wiki/Rapidly_exploring_random_tree (Accessed: 31 October 2023).