# SuboptimalSIPP

## Overview
This project contains an implementation of different versions of Bounded Sub-Optimal Safe Interval Path Planning algorithm as described in the [ICAPS 2020 paper](https://aaai.org/ojs/index.php/ICAPS/article/view/6674) ([arXiv version](https://arxiv.org/abs/2006.01195)).

Technically this project was forked from [AA-SIPP(m) project](https://github.com/PathPlanning/AA-SIPP-m/) and then additional functionality was added, i.e. three different suboptimal versions of SIPP were implemented (see the paper for more details): 
* Weighted SIPP with duplicate states and limited re-expansions (WSIPPd)
* Weighted SIPP with (unlimited) re-expansions (WSIPPr) 
* SIPP with FOCAL list (FocalSIPP)


Planning is carried out in (x, y, \theta) configuration space. Agents' headings, translating and rotating speeds, sizes are taken into account. Agents are considered to be open disks of predefined radii. Radius of each agent can be specified and can be any positive real number, e.g. some agents can be bigger than the grid cells. They can be smaller as well. "Open disks" means that when the distance between the agent of radius r_1 and the agent of radius r_2 equals r_1 + r_2 no collision occurs, the distance has to be less than r_1 + r_2 for the collision to occur. 

Agents' valid actions are (i) translate (ii) rotate in place (iii) wait in place. Moves' endpoints are tied to the centers of grid cells. Moves can be of arbitrary durations, i.e. the durations are not discretized into timesteps, e.g. duration of the translation action is moving speed (set by the user) times the length of the segment agent is traversing. Inertial effects are neglected so far, i.e. agents accelerate/decelerate instantaneously.

The instance to be solved (as well as algorithm's options) is supposed to be encoded in XML-file(s) of predefined structure (see __"Input and Output files"__ or [examples](https://github.com/PathPlanning/AA-SIPP-m/tree/master/Instances)) and passed to the solver as command line argument(s). The result (paths and some additional information) is output to the (distinct) XML-file as well.

Code is written in C++ and is meant to be cross-platform. Implementation relies on C+11 standard, STL and boost. Open-source library to work with XML (tinyXML) is included at the source level (i.e. .h and .cpp files are part of the project). Boost is required to support "multi-index" data structure.


## Getting Started

To go and try this algorithm you can use QtCreator or CMake.
Both `.pro` and `CMakeLists` files are available in the repository.

Notice, that project uses C++11 standart. Make sure that your compiler supports it.

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

**[Qt Creator](https://info.qt.io/download-qt-for-device-creation?hsCtaTracking=c80600ba-f2ea-45ed-97ef-6949c1c4c236%7C643bd8f4-2c59-4c4c-ba1a-4aaa05b51086)**  &mdash; a cross-platform C++, JavaScript and QML integrated development environment which is part of the SDK for the Qt GUI Application development framework.

**[CMake](https://cmake.org/)** &mdash; an open-source, cross-platform family of tools designed to build, test and package software.

**[Boost](https://www.boost.org/)** &mdash; open-source, cross-platform peer-reviewed portable C++ source libraries.

### Installing

Download current repository to your local machine. Use
```
git clone https://github.com/PathPlanning/AA-SIPP-m.git
```
or direct downloading.

Built current project using **Qt Creator** or **CMake**. For example, using CMake
```bash
cd PATH_TO_THE_PROJECT
cmake .
make
```

## Running
Make the algorithm work by launching the built executable with the command line argument specifying the location on the input XML-file that encodes the planning task. The result of planning in the form of the other XML file will appear in the same folder as input file and, by default, will be named `_log.xml`. See more about the input/output files below.

## Input and Output files

Both files are an XML file with a specific structure.

Note that all tags in XML files are case sensitive.

Input file should contain:
* Mandatory tag `<agents>`. It describes the task.
   * Optional tag `<defaultparameters>`. It is used to change the default values for size, speeds and headings of agents.
      * `size` &mdash; attribute that defines the radius of agents. Possible values [0, 10]. The value is relative to the grid's cell size, which always has conditional value 1. For example, 0.5 means that the agent size corresponds to the circle inscribed in the cell.
      * `movespeed` &mdash; attribute that defines the movement speed of agent. Possible values (0, 10]. The value is relative to the grid's cell size, which always has conditional value 1. For example, 1.0 means that the movement between two adjacent cells takes one time unit.
      * `rotationspeed` &mdash; attribute that defines how fast the agent can change its heading. Possible values (0, 10]. Speed 1.0 means that the agent can rotate for 180 degrees within one time unit. Other speeds are set as a coefficient to this ratio. This attribute makes sense only if option `<planforturns>` is active.
      * `start.heading`, `goal.heading` - attributes that defines the heading of agents. Possible values are [0, 360] degrees. Additional possible value for `goal.heading` is `-1` or `whatever` which means that goal's heading doesn't matter. Zero degrees corresponds to the direction of increasing of coordinate X. The angle value increases counterclockwise. These attributes make sense only if option `<planforturns>` is active.

      If any of the parameters doesn't specified, the default value defined inside the code (in gl_const.h) is used. The default values are the following: `size`=`0.5`; `movespeed`=`1.0`; `rotationspeed`=`1.0`, `start.heading`=`0`; `goal.heading`=`-1`.
   * Mandatory tag `<agent>` defines start and goal locations, size and speeds of one agent.
      * `id` &mdash; attribute that defines the identicator for agent. It is used only for notifications and can have any value. In cases when ID doesn't specified the number in the order of enumeration is used.
      * `start.x`, `start.y`, `goal.x`, `goal.y` &mdash; mandatory attributes that defines the coordinates of start and goal locations. Legal values for `start.x` and `goal.x` are [0, .., *width* - 1], for `start.y` and `goal.y` - [0, .., *height* - 1]. (0,0) coordinates correspond to the upper left corner of the grid. Two agents can't have equal start or goal location and the distance between their start and goal locations must be not lower than the sum of their sizes. Moreover these locations must be traversable with respect to the sizes of the agents.
      * `size`, `movespeed`, `rotationspeed`, `start.heading`, `goal.heading` &mdash; additional attributes that allow to change the values of these attributes for an exact agent. The values specified here have a higher priority than the values specified in the default parameters.
      
* Mandatory tag `<map>`. It describes the environment.
    * `<grid>` &mdash; mandatory tag that describes the square grid constituting the map. 
    * `height` and `width` &mdash; mandatory attributes of tag `<grid>` that define size of the map. Origin is in the upper left corner. (0,0) - is upper left, (*width* - 1, *height* - 1) is lower right.
    *  `<row>` &mdash; mandatory tags, each of which describes one line of the grid. Each `row` contains a sequence of "0" and "1" separated by blanks. "0" stands for traversable cell, "1" &mdash; for not traversable (actually any other figure but "0" can be used instead of "1"). Total number of "0" and "1" in each row must be equal to the width of the grid. Total number of rows must be equal to the height of the grid. 
    
* Mandatory tag `<algorithm>`. It describes the parameters of the algorithm. In cases when some tags, that describes the settings, are not specified or are incorrect the default values are taken.
    * `<algtype>` &mdash; possible values are `1` - WSIPPd, `2` - WSIPPr, `3` - FocalSIPP. By default the value is `1`.
    * `<weight>` &mdash; possible value is any real number in range `[1;10]`. Defines the weight of heuristic function. By default the value is `1.0`.
    * `<allowanyangle>` &mdash; possible values `true` or `false`. Defines the choice between AA-SIPP and SIPP algorithms. By default the value is `true`.
    * `<connectedness>` &mdash; defines the connectedness of the grid. Possible values: 2(4 neighbors), 3(8 neighbors), 4(16 neighbors) and 5(32 neighbors). By default the value is `2`.
    * `<timelimit>` &mdash; defines  the amount of time that the algorithm can spend on finding a solution. Can be helpful in cases of using rescheduling. Possible values: `-1` - no limit; `n` - number of seconds (n>0). By default the vaule is `-1`.
    * `<startsafeinterval>` &mdash; defines the size of additional constraints in the start locations of low-prioirity agents. Helps to find a solution for instances with many agents without rescheduling. Possible values: `0` - no startsafeintervals; `n` - the size of constraints, counts in conditional time units. By default the value is `0`.
    * `<planforturns>` &mdash; defines the option of taking into account the headings of agents and the time required to change them. Possible values `true` or `false`. The cost of changing the heading is defined by the attributes `rotationspeed` that were described above. By default the value is `false`.
   * `<waitbeforemove>` &mdash; defines additional delay that each agent performs before starting to move along the next section. Possible values are [0;100]. By default the value is `0`.
   * `<inflatecollisionintervals>` &mdash; this option increases the time between the moment when the agent and a dynamic obstacle pass through the same location. Possible values are [0;100]. By default the value is `0`.
   
* Optional tag `<options>`. Options that are not related to search.
    * `<loglevel>` &mdash; defines the level of detalization of log-file. Default value is "1". Possible values:
        - "0" &mdash; log-file is not created.
        - "1" &mdash; log-file contains the names of input files, short `<summary>`, `<path>` and found trajectories inside tags `<agent>`. `<summary>` contains info of the path length, number of steps, elapsed time, etc. `<path>` tag looks like `<grid>` but cells forming the path are marked by "\*" instead of "0". Each tag `<agent>` contains a path that consists of a sequence of sections with start and goal locations and duration. 
        - "2" &mdash; instead of names of the input files all the input data is copied to the log-file plus the log-info that were made by loglevel="1".
    * `<logpath>` - defines the directory where the log-file should be written. If not specified directory of the input file is used.
    * `<logname>` - defines the name of log-file. If not specified the name of the log file is: "input file name" + "\_log" + input file extension.
    
* Optional tag `<dynamicobstacles>`. Contains the trajectories of dynamic obstacles.
   * Optional tag `<defaultparameters>`. It is used to change the default size value. Note that move speed and rotation speed can't be modified as their exact values already sewn inside duration attributes of the sections. The same can be said about the headings. 
   * `<obstacle>` &mdash; contains the trajcetory of one dynamic obstacle represented as a seqence of sections. Can have its own `size` attribute.
      * `<section>` &mdash; describes a part of a trajectory. It must contain attributes `start.x`, `start.y`, `goal.x`, `goal.y` and `duration`. 
## Launch
To launch the application you need to have an input XML-file with all required information. If it's all-in-one file:
```
./SuboptimalSIPP initial_file_name.xml
```
Due to simplify the procedure of performing experiments with many tasks, the parts of input XML-file can be devided into several files:
* Task-file &mdash; contains part `<agents>`.
* Map-file &mdash; contains part `<map>`.
* Config-file &mdash; contains parts `<algorithm>` and `<options>`.
* Obstacles-file &mdash; optional file, contains part `<dynamicobstacles>`.

The application can be launched with separated input files in the following way:
```
   ./SuboptimalSIPP task_file_name.xml map_file_name.xml config_file_name.xml
```
or
```
   ./SuboptimalSIPP task_file_name.xml map_file_name.xml config_file_name.xml obstacles_file_name.xml
```
If `<loglevel>` has value `1` or `2` the output file will be placed in the same folder as input file and, by default, will be named `_log.xml`. For examlpe,
```
"initial_file_name.xml" -> "initial_file_name_log.xml"
```
In case of using separate input files the output file by default will be named as the task-file, i.e. `task_file_name_log.xml`.

## Multi-agents
Recall that this project was forked from [AA-SIPP(m) project](https://github.com/PathPlanning/AA-SIPP-m/). A nice by-product of this is that one can run sup-optimal SIPP in the prioritized multi-agent path finding mode. It this case some additional parameters should be set (like specifying other agents' start-goal locations etc.). Refer to the readmi of the original repo to sort it out.

[![Build Status](https://travis-ci.org/PathPlanning/SuboptimalSIPP.svg?branch=master)](https://travis-ci.org/PathPlanning/SuboptimalSIPP)
