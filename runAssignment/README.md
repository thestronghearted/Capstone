When running the multiple robots, this folder has been created to streamline the process

Before running these files, you first have to make within the build folder,
inside there run the following (if you have not done so already):

robogen-0.3.1/build$ cmake -DCMAKE_BUILD_TYPE=Release -G"Unix Makefiles" ../src

robogen-0.3.1/build$ make -j3

When running a single robot just use (with reference to path in relation to the build folder):

robogen-0.3.1/runAssignemt$ python3 runAssignment.py <robot file name> <configuration file>
eg: 
robogen-0.3.1/runAssignemt$ python3 runAssignment.py "../examples/cart.txt" "../examples/conf.txt"

When running multiple individual robot files create a text file that contains the names of all the robot-files and the number robots to be simulated:

eg. robotlist.txt
(contains)
[
robotNum <number of robots>
<robot file name (with reference to path in relation to the build folder)>
... (continue for all robots - heterogeneous otherwise just the one for homogoneouse)
] (do not include [])
eg.
[
robotNum 4
../examples/cart.txt
../examples/walkingStarfish.txt
../examples/cart.txt
../examples/starfish.txt
] (do not include [])

Then run the command with that as the file (with reference to path in relation to the build folder):
robogen-0.3.1/runAssignemt$ python3 runAssignment.py <list of robots file> <configuration file>
eg: 
robogen-0.3.1/runAssignemt$ python3 runAssignment.py "../examples/robotlist.txt" "../examples/conf.txt"

If a swarm json file is used, just use that as the reference (with reference to path in relation to the build folder):
robogen-0.3.1/runAssignemt$ python3 runAssignment.py <robot swarm file name> <configuration file>
eg. 
robogen-0.3.1/runAssignemt$ python3 runAssignment.py "../examples/robot_multiple.json" "../examples/conf.txt"