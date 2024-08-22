# IoT-Project-2024

<p align="center">
  <img src="https://fede3751.github.io/IoT-lectures-2024/imgs/project/project_splashart_2024.png">
</p>

Official repository for the IoT project 2024.

## Notes for development

We need:
- a new interface for the messages published from the sensors, something with a clock and maybe an expiration,
- a better algorithm to replace the cache entries when the cache is full,
- a mechanism to destroy expired cache entries (maybe using timers),
- improve the distribution of the creation of messages by the sensors (now something like 1 per x second),

moreover nothing is implemented concerning the requests of an ipotetical base station towards the baloons. Hypotheticali it should request data from specific sensors to the baloons. This can be done in different ways each with different level of difficulty:
1. via requests issued at a costant rate,
2. via requests issued following some kind of distribution,
3. via a real application (something that could be some kind of gui that allows the user to monitor the sensors)

clearly the last solution could be expanded in various ways. Furthermore all those solution should involve exchange of messages either via topic or service, niether of those are already implemented.

Lastly, we need to decide on the behaviour that baloons and/or sensor should follow. First open question: should they move?

### update 1
- the interface for the messages is done
- a simple LRU algorithm for removal of cache entries is in place
- a simple expiration mechanism is also in place

concerning the sensor's/baloon's behaviour it would be advisable a simple patrolling for the sensors and a static approach of the baloons possibly applying clustering to optimize their position in order to cover all the sensors

### update 2
- added poissonian distribution to spawn the sensors message

we need to decide the behaviour of the exchange Antenna-Balloon (via topic, via service, or via action?) probably the service is the best option however we created topics and an action interface. Keep in mind that theese interactions should be monitored so that the QoS can be evaluated and studied for the report.

There's still the need to create some kind of patrolling system for the sensors.

## Launching the Simulation

You can start the simulation displayed in the image with the command:

```
ros2 launch project_main simulation_launch.py
```

Please note that in order for resources to be correctly found by gazebo, you should not launch your simulation outside this workspace.<br><br>

## Packages Organization

The project containts two ROS Packages, you can start from them to build your solution:

**project_interfaces**<br>
  &emsp; Used to store the interfaces used for the project. Currently contains only the **Patrol** action.<br><br>
**project_main**<br>
  &emsp; Main package where all the code is stored. Your solution can start by building on top of this package, or in a new package alongside this.
  Having a side-package is suggested in order to prevent conflicts when new updates are pushed.


The file configuration to launch the simulation is **simulation_launch.py** inside the **launch** folder of **project_main**.<br><br>


## ROS Nodes

There are currently multiple nodes being launched:

**SimulationManager**:<br>
&emsp; Handles the main logic of the simulation. The behaviour of the range of the sensors and the tranmission between the various interfaces of the devices in the simulation is described here.
You currently have already implemented for you the bridge between the interfaces of the sensors and those of the balloons.<br><br>
**BalloonController**:<br>
&emsp; Used to describe the behaviour the balloons. The **Patrol** action server is advertised here, along with the subscription to movement and tranmission topics. There is currently no implementation to handle incoming packets from sensors, just a simple print message.<br><br>
**SensorController**:<br>
&emsp; Same as the controller for the balloons. The sensors currently output a simple string every second.<br><br>
**FleetCoordinator**:<br>
&emsp; Class which acts as an ActionClient to the balloons and coordinate their moving according to the demands.<br><br><br>

To help with the implementation of timed events, the **sim_utils.py** file contains an **EventScheduler** class which allows to schedule events based on simulation time.<br><br>

## Project Assignment

For the full assignment of the project, please refer to the main file here: <a href="https://fede3751.github.io/IoT-lectures-2024/misc_files/IoT_Project_2024_Assignment.pdf">IoT_Project_2024_Assignment.pdf</a><br><br>

Good luck!
