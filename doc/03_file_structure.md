# 3. File Structure

The ALR Simulationframework contains 5 top level directories:

- [alr_sim](../alr_sim): Here you can find basic level programming modules as well as robot control modules.  
You can use these to design your own task from scratch

- [doc](../doc): Various User Guides and Developer Documentations aiming to help understand the ALR SimulationFramework.

- [demos](../demos): Here you can find some examples on how to define a basic robot learning tasks. Understand all the procedures to define a robot, a scene, a camera system as well as different ways to control a robot.

- [envs](../alr_sim/envs): Here we provide an assortment of predefined Reinforcement Learning environments. They follow the [OpenAI Gym interface ](https://gym.openai.com/)and can be used to quickly deploy and benchmark some RL algorithms.  
You can also use them as a guide on how to design your own RL Gym Task. They are built by using the constructs provided by [alr_sim](../alr_sim), and some gym specific classes found in [alr_sims.gyms](../alr_sim/gyms)


- [models](../models): Here you can find the geometry, CAD meshes, textures and configuration files for the robot, camera and other objects. You can use them to define your own task.
	- If you want to create and define your own simulation objects, such as obstacles, bananas, please follow the examples there.

- [test_demos](../test_demos): This directory contains test procedures for various aspects of the simulations, such as physics parameters.  
They might serve as an interesting starting point for your own physics exploration, but are potentially deprecated, as they might not have been kept up to date during development.

[Back to Overview](./)
