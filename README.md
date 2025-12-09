# Muscle-activation-OpenSim-sEMG
Summary: This is a project in which we solved for upper body muscle activation during isometric movements and compare it to sEMG of 8 muscles in the arm. The forces are recorded from a BOTA force/torque sensor attached to a handle at the end effector of a KUKA robot.
Data: Project data is shared on google drive: https://drive.google.com/drive/folders/1Ve6RCtBJ3CAHE0h3tUTHv5pxKAgLz2R7?usp=sharing
Video presentation: https://drive.google.com/drive/folders/18RZmnztQqE8qIqR0mG6Sq6c2uKGPfMOM?usp=sharing 

# Force
Read the force and torque data from ROS bag and rotate it to be expressed in motion capture space: 
1. Open ROT_force.m.
2. Change the marker locations based on your marker set on the robot end effector.
3. Initialize the FilePath_read with the location of your ros bag.
4. Initialize FilePath_save with the location where you want to save the .sto files.
5. Make sure you have the writeSTO funstion in the same folder.
6. run the script.

Write .mot and XML files for force data for each trial:
1. Righ click on each .sto file, open Properties and change the ".sto" to ".mot".
2. Open Force_sim_Iso7 with Notepad++.
3. Change the datafile to match your .mot file.

# OpenSim 4.5
Download MOBL model version 4.1: https://simtk.org/frs/?group_id=657
This folder doese not contain Geometry folder, we pulled the Geometry from Arm26 model on opensim 4.5 examples.

Scale the model:
1. On OpenSim, load the model.
2. Open scaling tool window.
3. Load scaling setup from scaling folder on google drive.
4. Change the file path to load A_Iso_01.trc from scaling folder on google drive.
5. Run the scaling.
6. Save the saled model.
7. Close the model.

Run inverse kinematics:
1. Load the scaled model.
2. Open inverse kinematic tool window.
3. Load inverse kinematic setup from IK folder on google drive.
4. Change the file path to load the corresponding .trc file from Mocap folder on google drive.
6. Save the results in the desired Motion File path.
7. Run the inverse kinematics.

Run inverse dynamics:
1. Open inverse dynamics tool window.
2. Change the file path to load the the inverse kinematic results.
3. On the External Loads tab, load the corresponding XML file.
4. Turn on the 6 Hz filtering.
5. Save the results in the desired Directory.
6. Run the inverse dynamics.

# Model modification for static oprimization
Multiply muscle forces by 2:
1. Open ModifyModel.m
2. Change modelFile to match your scaled model file path.
3. (you can change 2 to other numbers based on your application or change the name).
4. Run the script.

Create reserved actuator:
1. Open cmc_actuators_wrist.xml with Notepad++.
2. Modify the optimal_force based on your application.

# Static optimization
1. Load the scaled model.
2. Lock these coordinates: (rx ry rz tx ty tz)
3. Open the static optimization tool window.
4. Load the inverse dynamics results.
5. On the Actuators & External Loads tab, in External Loads space add the corresponding .xml force data.
6. And in Actuators space add the .xml file corresponding to the reserved actuators.
7. Save the results in the desired Directory.
8. Run the static optimization.

# sEMG
1. sEMG 
