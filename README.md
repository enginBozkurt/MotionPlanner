# MotionPlanner
## Motion Planner for Self Driving Cars


The goal of this project will be to have a **functional motion planning stack** that can **avoid both static and dynamic obstacles** while tracking the center line of a lane, while also **handling stop signs.** To accomplish this, we will have to implement behavioral planning logic, as well as static collision checking, path selection, and velocity profile generation.

## General Overview and some key concepts
![Screenshot_5](https://user-images.githubusercontent.com/30608533/63599391-29641000-c5ca-11e9-9a7d-e58788e5838b.jpg)

## Implementing the Motion Planner
In this project, we will be editing the **"behavioural_planner.py", "collision_checker.py", "local_planner.py", "path_optimizer.py", and "velocity_planner.py"** class files (found inside the "PythonClient\Course4FinalProject" (for Windows) or "PythonClient/Course4FinalProject" (for Ubuntu) folder). This is where we will implement our motion planner. There are 5 main aspects of the planner you will need to implement, **behaviour planning logic, path generation, static collision checking, path selection, and velocity profile generation**

## Solution Approach

### Path Generation

![Screenshot_1](https://user-images.githubusercontent.com/30608533/63599898-2ddcf880-c5cb-11e9-9e9f-9db5396b37e5.jpg)

<p>
   We had given you all of the required code to optimize a spiral to a given point. So all we needed to add was to extract the goal points from the way points sequence you were given. We did this by iterating through the way points and finding the closest one to the ego vehicle, and then progressing through way points until we arrived at a way point that was at least a look ahead distance away from us. The look ahead distance represents how far ahead of the vehicle we are planning during any cycle. Once this goal point is selected, we laterally offset additional goal points from the selected way point to generate a goal state set. We do this to give the car multiple options to select from, which will allow it to avoid obstacles. The lateral spacing is 90 degrees rotated away from the heading of the road at the target way point. This ensures that our planned path set conforms to the structure of the road. We then have to rotate and translate these points from the global frame to the ego vehicles frame, such that the ego vehicle is at the origin with zero heading. Once this is done, we can pass these goal points to the spiral optimizer which will then compute the path that we can use in our future planning steps.
  
</p>

### Collision Avoidance

![Screenshot_2](https://user-images.githubusercontent.com/30608533/63600422-371a9500-c5cc-11e9-9524-e4cdc2f11e8b.jpg)

<p>
   Once we have our path set, the next step to perform is collision checking and path selection. To compute the best feasible and collision-free path for our ego vehicle to follow. The way we performed collision checking was to use the circle based method we introduced in Module Four. In particular, we overlaid circles on top of the vehicle location at each point along the path. This generates a set of circles which gives us a conservative approximation of the ego vehicle swath. We then checked if each of the obstacle points was located within any of the circles along the path. If any of the points did fall within the circles, then this path was labeled as having a collision. We then repeated this process for each of the paths removing all of those that collided with obstacles from future consideration. Once our collision check was complete, we then selected the collision-free path whose end-point was closest to the line center line as our best path. We select this because it allows us to track the global way points along the center of the lane even if we had to veer off course temporarily due to an obstacle along our path. This choice does lead us to cut as close as possible to obstacles. So we had to be careful to ensure our collision checks were conservative with some buffer.
</p>

### Velocity Profile Generation

![Screenshot_3](https://user-images.githubusercontent.com/30608533/63600434-3c77df80-c5cc-11e9-8845-042bee7af9fe.jpg)

### Behaviour Planning

![Screenshot_4](https://user-images.githubusercontent.com/30608533/63600441-413c9380-c5cc-11e9-9d7f-e11618b2f996.jpg)

## Final Results

![Screenshot_11](https://user-images.githubusercontent.com/30608533/63600723-c58f1680-c5cc-11e9-97e3-e299142ddca9.jpg)

![trajectory](https://user-images.githubusercontent.com/30608533/63600798-e9eaf300-c5cc-11e9-941f-bb0437b7b519.png)


![forward_speed](https://user-images.githubusercontent.com/30608533/63600935-2ae30780-c5cd-11e9-9687-4288d4f2ab38.png)
![steer_output](https://user-images.githubusercontent.com/30608533/63600936-2e768e80-c5cd-11e9-824d-59779d0a4e54.png)
![throttle_output](https://user-images.githubusercontent.com/30608533/63600951-33d3d900-c5cd-11e9-9ecd-8c8db06de30d.png)
