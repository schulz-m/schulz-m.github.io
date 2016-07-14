---
layout: page
title: About
---

Born in Berlin and raised up mainly in Zurich, where I have studied at [ETH Zurich](https://www.ethz.ch/en.html) for my Masters in Mechanical Engineering. Having worked and studied in various countries and industries around the world, I decided to pursue my biggest interest in robotics and I am currently working at [Rapyuta Robotics](http://www.rapyuta-robotics.com/) to enable the robotic revolution!

----

## Projects

Most of my public projects so far were in the research area of robotics, though I will try to push for some open source projects in the future that the community can hopefully profit from. 

### Pegasus Walking Robot

In my bachelors, I was a team member of [Project Pegasus](http://www.pegasus.ethz.ch/index.html), where the aim was to build a fast running, dynamic quadruped that follows a speed lane. The technical work included developing a simulation Framework in Matlab using Contact Dynamics, adapting the original control algorithms from [Marc Raibert](https://books.google.ch/books?id=EXRiBnQ37RwC&redir_esc=y) and developing State Estimators using the well known Kalman Filter framework in C++ on ROS.

### Robust State Estimate for Quadrotors

This was a 4 month project in my masters that was aimed at developing robust state estimates for a quadrotor using the least amount of sensors possible to enable a smooth landing. An important concept was the so called (and now widely used) [error-state Kalman Filter](http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf) and a ground estimation using barometer, sonar sensors and accelerometers. Some of the developed concepts could later be reused to [throw a quadrotor into the air](https://www.youtube.com/watch?v=pGU1s6Y55JI).

### High-Speed Flight of a Tethered Quadrotor

The Master Thesis was about inventing a method to enable high-speed, steady flights in confined spaces using a tethered quadrocopter. For this purpose, the centripetal forces exerted by the tether were used to fly circular trajectories with velocities up to 15 m/s and centripetal accelerations of more than 13 g in steady Flight. The achieved flight performance allows to record data that enables the characterization of quadrocopters at high airspeeds. The main contribution in this project was the system design and implementation on a [pixhawk microcontroller](https://pixhawk.org/) including the mechanical modelling of the system and the design of linear control strategies that made the [quadrotor fly up to its limits](https://www.youtube.com/watch?v=iJPy1geXu4M)!

### Building a Rope Bridge with Flying Machines

During my time as a research associate working at the [Flying Machine Arena](http://flyingmachinearena.org/), I had the honor to work on the development of methods and strategies that enable small flying machines to assemble a
rope bridge that can support the [crossing of a person](https://www.youtube.com/watch?v=CCDIuZUfETc). The work involved tuning the spool controller that deployed the ropes, implemented complex motion primitives and identify rope models to plan the building process.

## Photohraphy

Regarding online presence in my free time, I like to play around with design tools and photography, you can find some of my work on my [Flickr account](https://www.flickr.com/photos/144925784@N04/).

> Note: The source of this website can be found on my [github account](https://github.com/schulz-m/schulz-m.github.io).