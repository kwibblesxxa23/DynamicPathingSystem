# The Dynamic Pathing System
a dynamic pathing system for making autonomous functions using lemlib easyer. type your auton in a text file and have the bot do the rest. it also has support for actions during your auton such as intakes or piston functions<br>

# Includes
#include "dynamic-pathing-system.hpp"<br>
#include "dynamic-pathing-system.cpp"<br>

i suggest including this for an easyer codeing experience<br>
using namespace DynamicPathingSystem;<br>

you also need this guy<br>
DynamicPathingSystem::Pathing pathingObj;<br>

# Making A Path File
first you need a micro SD card to add the files to.<br>
the brain will get the file from the micro SD card.<br>
***importatnt note***<br>
***the micro SD card must be under 16 gigabytes and be formatted in fat32***<br>

once you have an SD card you can start making the path<br>
start with a text file -> "path.txt"<br>

inside the text file you will make your auton point by point<br>
your points should follow this formant<br>
X Y Heading forwards actions<br>

X - the X coord using lemlibs coord system<br>
Y - the Y coord using lemlibs coord system<br>
Heading - the heading in degrees using where the audiance is 180 (high stake side)<br>
forwards - a bool 1/0 signifying forwards/backwards respectively<br>
actions - basically the different actions your implimenting. <br>

the current ones in the code are as follows<br>
intake mogoMech doinker redirect<br>
intake - a number that can be 0 (off), 1 (forwards), 2 (backwards)<br>
mogoMech - bool for solenoids<br>
doinker - bool for solenoids<br>
redirect - bool for solenoids<br>

you can modify the actionItems code to use different items. this can be seen below (the bottom)<br>

# Using A Path
start with your "path.txt" file<br>
include it in your code like so<br>
const std::string path = "/usd/path.txt";<br>
make sure to include the file directory "/usd/" to the start so the brain can find it<br>

then you will need to make your action items like so, giving all of your motors/solenoids over to the program to use<br>
ActionItems actionItems(intake, topIntake, mogoMech, doinker, redirect);<br>

to actually use your path you need to include your files<br>
pathingObj.DynamicPathingSystemRun(const std::string& path, lemlib::Chassis& chassis, float rpm, float wheelDiameter, actionItems);<br>

an example on how you would use it<br>
pathingObj.DynamicPathingSystemRun(path, chassis, 360, 2.75);<br>

# future updates
better doccumentation as it is out of date and cleaner code as its way too messy.<br>
