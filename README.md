# Sensor-CAN_Hornet-XI_Electrical

Code for implementing and testing CAN communication between the Sensors PCB (Teensy) and the main SBC (Jetson Orin Nano) of the Hornet XI AUV

Uses the FlexCAN_T4 library on the Teensy side to send 7 CAN frames over the CANBUS of the AUV, at a bitrate of 1Mbp/s
Testing code on the jetson picks up the IDs, reconstructs the data and prints it out to the terminal in real time for debugging
