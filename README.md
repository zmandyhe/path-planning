# Self Driving Car: Path Planning
Self-Driving Car Engineer Nanodegree Program

### Goals
In this project my goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. I will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

![alt-text-1](https://github.com/zmandyhe/path-planning/blob/master/cover.png)

### Compliation and Run 

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Reflection

### Path Generation
- Use the sensor fusion data [ id, x, y, vx, vy, s, d] to track all the other cars on the same side lanes with the ego car, then loop into each of these cars to get the car lists that are in my current lane, in my left lane, or in my right lane.
- If a car in in my current lane, then evaluate the need of changing lane (my_ref_lane) or slowing down (my_ref_vel) with the following criterias:
    - the car is in the same lane, and distance with the ego car is less than 30m,  it conclude the need to change lane or slow down (bool change_lanes_or_slow_down = True);
    - check to perform lane changing or speed slowing down with the following considerations:
        -  consider lane changing first: check if the car on the left or right lane impedes my ability to swith lanes ( bool dont_go_left,   bool dont_go_right). If any of the below criterias is met, the ego car must stay at the current lane and slow down.
            - Its distance is less than 30m and it is ahead me and slower than me
            - It is closer than 15m behind me and faster than me
            - it is very closer to me, the distance to me is than 10m 
            - already in a right-most or left-most lane (my_ref_lane == 0, my_ref_lane == 2)
            - if my speed is less then 20mph
            - 
        - Otherwise: 
            - when (!dont_go_left = Ture), then change to left lane (my_ref_lane = my_ref_lane - 1);
            - when (!dont_go_right = Ture), then change to right lane (my_ref_lane = my_ref_lane + 1).
    
### interpolate Waypoints with a Spline to Create smooth trajectories
- Use the waypoints starting from the previous two points returned from the map; if there aren't previous points, use the current waypoint returned from the simulator as the starting reference
- Transform from Frenet s,d coordinates to Cartesian x,y (getXY from helper funtion), add evenly 30m spaced points ahead of the starting reference control points 
- Track 30m ahead of me in x position and transform to the target Euclidean distance (target_dist) and number of points (num_points) to fill in the spline according to my velocity (my_ref_vel) in 0.2s interval.

## Key Performance Evaluation
Below are key merits and some code snippets to ensure the car safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.

### The car drives according to the speed limit
I put a speed limit vanity check under the lane change or slow down section.

```
else if (my_ref_vel < speed_limit) {
            my_ref_vel += .224;
```

### Max Acceleration (10 m/s^2) and Jerk (10 m/s^3) are not Exceeded


### Car does not have collisions
Check closely if other cars is in my lane (if yes, change lane or slow down) or is emerging to my lane.

```
bool other_is_in_my_lane  = (other_lane == my_ref_lane) || (other_merging_lane == my_ref_lane);
```

### The car stays in its lane, except for the time between changing lanes
Change lanes only when there is leading car ahead of me in my cane.
And the car becomes an obstacle as it is slower.

```
change_lanes_or_slow_down = change_lanes_or_slow_down || (other_is_ahead && other_is_in_my_lane);
```
```
bool other_is_obstacle = ( (other_is_ahead && other_is_slower)
                        || (other_is_behind && !other_is_slower) || other_is_close);
```

In general, set don't go left and don't right as true so that the ego car stays in the current lane.
```         
bool dont_go_left = true;
bool dont_go_right = true;
```

```
dont_go_left  = dont_go_left || (other_is_left && other_is_obstacle) || (min_dist_left < min_dist_here;
dont_go_right = dont_go_right || (other_is_right && other_is_obstacle) || (min_dist_right < min_dist_here);
```
### The car is able to change lanes
When there is no obstacles on my left or right lane, and I have to change lane.
```
} else if (!dont_go_left) {
    // if left lane free, go there 
    my_ref_lane = (my_ref_lane - 1);
} else if (!dont_go_right) {
    // if right lane free, go there
    my_ref_lane = (my_ref_lane + 1);
}
```

## Input Data
Here is the data provided from the Simulator to the C++ Program.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates
["y"] The car's y position in map coordinates
["s"] The car's s position in frenet coordinates
["d"] The car's d position in frenet coordinates
["yaw"] The car's yaw angle in the map
["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along the path has processed since last time. 
["previous_path_x"] The previous list of x points previously given to the simulator
["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value
["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Implementation Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. 

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```
