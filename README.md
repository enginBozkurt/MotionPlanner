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

### Collision Avoidance

![Screenshot_2](https://user-images.githubusercontent.com/30608533/63600422-371a9500-c5cc-11e9-9524-e4cdc2f11e8b.jpg)

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
