# robot_diffusion
Used ARGoS to simulate diffusion of multiple robots.

10 foot-bots perform obstacle avoidance while navigating in an small square environment. The robots are randomly distributed in the environment. The simulation is inspired from the Diffusion 10 example given [here](https://www.argos-sim.info/examples.php). I have changed the logic of obstacle avoidance.

I have tried to implement the controller using a switch case, where the case denotes which direction the robot will move. The direction of robot's motion is calculated using a simple function (int obstacle_decide_motion_direction) that uses the proximity sensor readings. Four cases are considered for movement: straight (0), Left (1), Right (2), and Reverse (3).
