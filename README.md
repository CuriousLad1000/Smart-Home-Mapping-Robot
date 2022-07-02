# Smart home mapping Robot

## Introduction

Smart autonomous robots are becoming an indispensable part of our lives.
Robots are already part of our lives without us even realizing it. From
cars to Factories to Medical care they are playing an important role in
providing improved product quality cheaper products and better
healthcare. Even the smart vacuum cleaner in our house that roams around
while cleaning the entire house is an autonomous robot that is tasked to
navigate around the house and clean it.

## Project Objective

In this project, we are going to see implementations of two different
kinds of techniques through which we can control our unicycle robot and
also build a scaled model of the house and use the unicycle robot to
create a SLAM map. We are going to simulate (not completely but the idea
is there) a smart vacuum cleaner type robot that maps the whole house to
navigate autonomously.

**Tasks**

-   To achieve our objective, we need to <b><u>create a scaled model of the
    house]</u></b> and use Gazebo and ROS to create a SLAM map. We first
    create a structured model in Gazebo using the real dimensions of my
    house.

-   We are going to Implement the <b><u>Unicycle variant of the Collision
    avoidance system</u></b> with various controller modes such as
    go-to-goal; Obstacle avoidance; Wall-following etc\...

-   The Robot will be using the <b><u>Layered architecture</u></b> where
    the model will be fed various waypoints assigned inside the home for
    the Trajectory tracker to reach the desired destination. By using
    this technique, we will make our robot navigate inside the house and
    map it without colliding with walls or objects inside.

-   Furthermore, we are also going to <b><u>implement another kind of
    controller</u></b> which will control our Unicycle robot by
    segregating the Translation and Rotation tasks.

*Full list of features and capabilities are listed in a separate section
below.*

## Software used

-   ROS noetic with Gazebo for designing home model and simulating world
    physics for the robot.

-   Jupyter notebook, Notepad++ for Python programming.

-   Various tools provided by ROS package to test, debug and plot
    various features.

## Features offered

-   Two different kinds of controlling mechanisms are implemented and
    compared.

-   Robot's controller has Obstacle avoidance built in and is able to
    follow along the boundary of the object.

-   Robot detects the obstacle using LiDAR.

-   Robot follows the provided list of waypoints to complete objective.

*For more details, please refer to the project's wiki*

## Simulation images

**Test field created in Gazebo**

<br/>

<p align="Center">
  <img width="1189" height="599" src="/osama.tasneem/Smart-Home-Mapping-Robot/wiki/raw/images/d0e7e3e2e0d4380aefe377819b61b182c4c86d72.png">
</p>

<br/>
<br/>

**Model of House**


<br/>

<p align="Center">
  <img width="640" height="380" src="/osama.tasneem/Smart-Home-Mapping-Robot/wiki/raw/images/01858bdf7e55929564d2d928421c0808aa5c5128.png">
</p>

<br/>
<br/>

**SLAM map created by the robot**


<br/>

<p align="Center">
  <img width="640" height="380" src="/osama.tasneem/Smart-Home-Mapping-Robot/wiki/raw/images/5c701e6b1f02a3e5e281e2a4a1683950c9692b07.png">
</p>

<br/>
<br/>

**Simulation**

> **Open Field Test**

<br/>

<p align="Center">
  <img width="800" height="450" src="/osama.tasneem/Smart-Home-Mapping-Robot/wiki/raw/images/open_field.gif">
</p>

<br/>
<br/>

> **Home Mapping**

<br/>

<p align="Center">
  <img width="800" height="450" src="/osama.tasneem/Smart-Home-Mapping-Robot/wiki/raw/images/Uwf.gif">
</p>

<br/>
<br/>

> **Trans Rot controller**

<br/>

<p align="Center">
  <img width="800" height="450" src="/osama.tasneem/Smart-Home-Mapping-Robot/wiki/raw/images/SLAM.gif">
</p>

<br/>
<br/>
