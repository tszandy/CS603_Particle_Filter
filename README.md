# CS603_Particle_Filter
Particle filter for mobile robot localization using a laser rangefinder.

## Dependencies
Install [ROS](http://wiki.ros.org/ROS/Installation)

## Clone This Repository
   ```
   git clone --recurse-submodules [REPO URL]
   ```
   **IMPORTANT:** The `--recurse-submodules` is required to clone the shared
   library submodule.

## Build Instructions
1. Clone this repo, and enter the directory. 
   All subsequent commands must be run from within the directory.
1. Add the project to the ROS packages path:
   ```bash
   export ROS_PACKAGE_PATH=/PATH/TO/YOUR/REPO:$ROS_PACKAGE_PATH
   ```
   Pro-tip: add this line to your `.bashrc` file.
1. Run `make`.  
   DO NOT RUN `cmake` directly.  
   DO NOT RUN `catkin_build` or anything catkin-related.  
   
## Run 
1. To run the particle filter with a bag file named `myfile.bag`:
   ```
   ./bin/particle-filter --input myfile.bag
   ```
1. To visualize the particle filter in RViz:
   ```
   rosrun rviz rviz -d particle-filter.rviz
   ```

## Example Bag Files
An example bag file for you to test with can be downloaded [here](https://drive.google.com/file/d/1kV19baRDWPyfWqOiFUMmgpJGSMssmtAm/view?usp=sharing). Please make sure you do not include the bag file with your submission.
