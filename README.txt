—-—-—-—-—- USAGE INSTRUCTIONS —-—-—-—-—- 
The GLFW library should be available (installed on Titan).

"make all" will compile all project code. Individual make targets are also available for each part:

make prm-test - compiles PRM GPU program make prm-test-cpu - compiles PRM CPU program make visualize
- compiles ray trace GPU program make visualize-cpu - compiles ray trace CPU program make display -
compiles display program

All executables are compiled into /bin. Each executable is described below.

—- PRM GPU program —- 
Usage: prm-test  

Outputs data to the terminal consisting of the stages of the PRM/path generation, the nodes that
compose the path, a set of intermediate nodes between the elements of the path to help display the
movement when visualized, and the corresponding locations of the links in 3D space with where each
link starts and ends. This program also outputs a text file giving the number of links in the robot,
the number of nodes total, and then these 3D link locations which can be utilized by the visualizer.

—- PRM CPU program —- 
Usage: prm-test-cpu 

Identical to PRM GPU program, but does not use the GPU

—- Ray trace GPU program —- 
Usage: visualize <input> [output] 

Takes input data produced by a PRM program to generate 2D images of the 3D space defined in the
data. If an output file is provided, these images are output as plain text pixel values.

—- Ray trace CPU program —- 
Usage: visualize-cpu <input> [output]

Identical to the ray trace GPU program, but does not use the GPU

—- Display program —- 
Usage: display <input>

Takes input pixels produced by a ray trace program and displays the images in sequence to produce an
animation. The ENTER key can be used to pause and resume the animation.

Compiled executables are included. Note that all input/output should be located in the project root
folder.

—-—-—-—-—- PROJECT DESCRIPTION —-—-—-—-—-

The project comes in two major parts - the PRM generation and planning, and the ray tracer.

One set of files handles generating the PRM nodes for a 3 DOF robot (consisting of a pan joint and
two tilt joints where the links of the robot exist in three dimensional space) and then planning a
path around a set of spherical obstacles using the A-star algorithm. This is handled in prm_gpu.cu. 

For the other part, the main files are visualize.cpp/visualize.cu and display.cpp. The first two
programs execute the same task - given a 3D space defined by a set of links, use ray tracing to
generate a 2D image. Multiple positions (sets of links) can be provided as input, in which case
these programs will generate a sequence of 2D images. More specifically, these programs use the
given positions of each link to generate a sphere at each endpoint and create a set of triangles
that form a cuboid to connect each endpoint. Then, given a (hard-coded) camera and image plane, ray
tracing is used to determine the color of each pixel, detecting collisions with the triangles and
spheres and filling in the color of the closest intersection. For improved depth perception, a basic
distance-based brightness adjustment is used. The resulting set of 2D images are then written to a
file to be used by display.cpp. This program takes in a series of 2D images and displays them in an
OpenGL window. It also caps the frame rate and ensures a minimum animation length, and allows the
user to pause the animation.

—-—-—-—-—- RESULTS —-—-—-—-—-

—- PRM program —- 
When running the GPU or CPU versions of the code, we output the following key
information to the terminal: the path built (represented as joint theta values), the intermediate
joint locations, and the locations of the links at each intermediate node. Going through these in
order, the path built is the set of nodes that led to a connection from the start to the goal. In
terms of the number of nodes, this path itself is very short as once the arm is able to navigate
around the obstacle, it can connect right to the goal. This is shown in showing_path.png. Then, in
order to visualize the path, we need to effectively generate “frames” of the movement. As such, we
make intermediate joint locations to showcase how the robot would reach the next node in the path
and an example of a few of the intermediate joint locations is displayed in intermediate_nodes.png.
Finally, we reinterpret these joint locations as the locations of the links in 3D space. Each node
consists of two links where a link is represented as its starting and ending point in 3D space. For
example, the first node is represented with the first link starting at position (0, 0, 0) and ending
at (0, 1, 0) and the second link starts at (0, 1, 0) and ends at (0, 2, 0). Examples of how this
looks are shown in showing_links.png. Finally, we output these link locations to a text file to be
utilized by the visualizer. For the GPU code, this is placed in prm_test_output.txt and for the CPU
code it is placed in prm_test_cpu_output.txt.

—- Ray trace and display program —- 
The ray tracing programs should produce output files
representing images as plain text pixel values. The display program should open a window and display
an animation of the images in the input file. For example, a sequence of images from the same
animation is provided in the screenshots display1.png, display2.png, display3.png, display4.png, and
display5.png. These screenshots show a (successful) maneuver of a robot arm around obstacles to a
new position, with each screenshot spaced ~35 frames apart.

—-—-—-—-—- PERFORMANCE ANALYSIS —-—-—-—-—-

—- PRM program —- 
To start, when checking the run time of the GPU and CPU scripts, we see roughly a
23 times speedup in compute time as shown in prm_gpu.png and prm_cpu.png. Going into detail, the
three functions that were accelerated on the GPU were creating N nodes, connecting the nodes to K
neighbors, and running the A* planner. N and K were chosen to be 500 and 100 respectively to
accentuate the value of parallelization. The first of these was parallelized with the generation of
random points happening with the cuRAND library and each node was initialized through a parallelized
process. This function was especially optimized with shared memory and avoiding bank conflicts
through padding and an appropriate stride. The connecting K neighbors process was especially slow on
the CPU due to the nature of iterating through each of the nodes and for each node, trying to
connect K neighbors. Through parallel computation where each node could be evaluated individually,
this process was significantly faster on the GPU. Finally, for the A* search algorithm, the
computation was parallelized by computing the initial cost heuristic in parallel. Given that the
path needed to the goal wasn’t particularly long, this process wasn’t significantly faster than the
CPU implementation.

With additional time, a few more aspects can be improved. The slowest function of these is
connecting the K neighbors and with the setup of the computation, where when a neighbor is found
both the node and the neighbor node need the opposite added to their list of neighbors, a lot more
__syncthreads(); need to be run. By instead doing this with shared memory and instead filling the
lists with booleans of whether another node is a neighbor, one could prevent this concern of race
conditions and obtain faster computation. As for A*, the primary way this could be improved is by
parallelizing the logic of a priority queue. However, in order to see the real benefits of a faster
A*, more complex environments would need to be set up to cause the number of nodes needed for a path
to increase.

—- Ray trace program —- 
First, note that writing the images to the output text files is quite an
expensive operation, and thus we omit this process when comparing run times, since this was not the
focus of the project (accomplished by not providing an output file when running the program).
Comparing visualize and visualize-cpu, we see that when we produce 512x512 images from a sequence of
151 arm positions, each with 2 links, we achieve an approximately 213 times speedup (see
visualize_gpu.png and visualize_cpu.png for timing output). This was done by parallelizing the
computation of pixel colors across all pixels, since the ray tracing logic for a single pixel is
completely independent. We divide the image into 64x64 segments of pixels (note that we assume the
size of the images to be some multiple of 64 in each dimension) and pass each to a block of
dimension 64x16. Each thread in the block then computes the pixel values for 4 pixels. 

With additional time, the main improvement would be to fix the optimized kernel, which uses shared
memory, along with padding to ensure that there are no bank conflicts, and loop unrolling in each
thread. More peripherally, we would have liked to include a more sophisticated lighting model, which
may have allowed the optimization to make even greater gains. 

