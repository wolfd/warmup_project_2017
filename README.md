# Computational Robotics Warmup Project
## Danny Wolf and Jonah Spear

The goal of this project was be to program a Neato (a small vacuum cleaner robot) to execute a number of behaviors. In the process of implementing these behaviors, we learned about tools and strategies for debugging robot programs. This assignment was an opportunity to become familiar with ROS's publisher-subscriber architecture, work with sensor data (lidar, bump sensor, odometry), and use effective visualizations for debugging purposes.

### The Square Dance

The first behavior we implemented was that of the robot driving in a 1 meter x 1 meter square. We had the option to measure this distance using either timing or odometry. We chose to use odometry due to its higher accuracy, as well as the greater potential for learning. Using a proportional controller for the velocity of the robot and it's rate of rotation, we were able to specify movement commands to the robot with relatively high precision, although this approach was still limited by the innate innacuracies of odometry. This code can be viewed at square_dance.py

![Square Dance Gif](https://github.com/wolfd/warmup_project_2017/blob/master/screenshots/square.gif)

### Wall Follower

The goal of this behavior was for the robot to follow a roughly linear wall. To do this we calculated the angle of the robot with respect to the wall (defined by two reference points), and used a proportional controller to keep the angle at 90 degrees - and the robot parallel to the wall. 
![Wall Following Diagram](https://github.com/wolfd/warmup_project_2017/blob/master/rescources/SimpleWallFollowingDiagram.png)

This approach works well, however over long periods of time the error can accumulate and cause the robot to drift closer and farther from the wall. This is because we are controlling the angle from the wall, and not the distance to the wall. A more advanced approach could keep this distant as well, ensuring that the robot would not drift. Our approach, however, worked more than well enough for our purposes. The code for this can be viewed at wall_follower.py

![Wall following gif](https://github.com/wolfd/warmup_project_2017/blob/master/screenshots/wall-follower.gif)

### Person Follower

The intended behavior of this program is for the program to follow you around like a pet! This works by using lidar data to detect points within a certain box in front of the robot. The robot then rotates such that it is on axis with the centroid of the points, and moves such that it is a specified distance from that centroid. 

![Person Following Diagram](https://github.com/wolfd/warmup_project_2017/blob/master/rescources/PersonFollowing.png)

This approach works very well for people, however the robot can easily become confused when it detects rigid points such as a chair or a wall. Going further, one might take the derivative of the points, such that only points that are changing will be used to calculate the centroid. One important feature of our code was that we were able to seperate the centroid-finding and person-following behavior into two seperate nodes. person_finder.py transmits visualization data to rviz as well as the position of the centroid. person_follower.py takes the centroid and adjusts it's position to maintain a specified distance.

![Gif of person following](https://github.com/wolfd/warmup_project_2017/blob/master/screenshots/person-follower.gif)

### Obstacle Avoidance

The final portion of this assignment combined all the previous aspects in a much more complex task. The goal is for the robot to progress towards a goal, avoiding any obstacles it may encounter along the way. 

![Obstacle Avoidance Diagram](https://github.com/wolfd/warmup_project_2017/blob/master/rescources/ObstacleAvoidance.png) 

Rather than using a brute force approach (turn 90 degrees when it encounters an obstacle, and then turning back toward the goal once the obstacle is gone) we decided to implement a more general approach using potential fields. The basic idea is that each obstacle acts as a repellant, and the 'goal' acts as an attractor. The goal and the obstacles each generate a vector field. The robots movements is decribed the by the sum of the vector fields at the position of the robot. For a more detailed description of the approahch see [here](http://phoenix.goucher.edu/~jillz/cs325_robotics/goodrich_potential_fields.pdf). The relevant code is included in obstacle_avoider.py

![Gif of Obstacle Avoidance](https://github.com/wolfd/warmup_project_2017/blob/master/screenshots/obstacle_avoider.gif)


