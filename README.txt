Below is a list of the pertinent folders and what they include.

Collision_Avoidance includes the code needed for the collision avoidance algorithm. Run the Spider_track_v4 file in order to start the algorithm. This python scrpit is used to track the robots and is used with each of the following ino files. In order to run a simple collision avoidance test, load the agents with the new_movement algorithm and load the modem panstamp with the Modem ino file.

The IR_Collision_Avoidance includes the ino file necesary to run the infrared collision avoidance algorithm. This should be loaded on to the agents while the computer runs the Spider_track_v4 file. The IR_Simple_Avoid should be adequate for most situations. In order to disable the overhead collision avoidance and just utilize the IR avoidance simply comment out the setpoints = getSetpint(...) and uncomment out the following line. The panstamp connected to the computer should be loaded with the IR_Collision_Avoidance_Modem ino file.

In order to run the object manipulation, follow the pattern of the previous two algorithms. Simply load the new_manipulation ino into the agents and the object_manipulation_modem into the modem. This time instead of running the spider_track python script run the object_manipulation_camera script instead.

The panstamp_motortest file is a simple test to check and see if the motors in the agents are working correctly. Simply load the agents with the file and ensure each directional motor is moving one at a time. 