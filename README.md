# DStarLite
D* Lite implementation in C#
### What is D* Lite
D* Lite is a dynamic version of A*. It dynamically finds the shortest path to the goal.
### Uses of This Implementation
To navigate a robot to some goal coordinates, when the map gets explored as the robot moves. The map needs to be able to be represented as a grid with navigable and unnavigable terrain. The robot is able to move forwards, backwards and to the sides. It is not able to move diagonal. To allow diagonal movements or to add different cost to different movements some simple changes to the code need to be made.
### How to Use it
Create a class that inherits the DStarLiteEnvironment interface. This class is responsible for the interaction between the environment and the algorithm. It has two methods MoveTo and GetObjectsInVision which are pretty self-explanatory. See <a href="https://github.com/Bastiantheone/DStarLite/blob/master/TestProgram.cs">TestProgram.cs</a> for an example.
