# The Dynamic Pathing System
a dynamic pathing system for making autonomous functions using lemlib easyer. type your auton in a text file and have the bot do the rest

# Includes
#include "dynamic-pathing-system.hpp"
#include "dynamic-pathing-system.cpp"

i suggest including this for an easyer codeing experience
using namespace DynamicPathingSystem;

you also need this guy
DynamicPathingSystem::Pathing pathingObj;

# Making A Path File
first you need an SD card to add the files to.
the brain will get the file from the SD card.

once you have an SD card you can start making the path
start with a text file -> "path.txt"

inside the text file you will make your auton point by point
your points should follow this formant
X Y Heading forwards

X - the X coord using lemlibs coord system
Y - the Y coord using lemlibs coord system
Heading - the heading in degrees using where the audiance is 180 (high stake side)
forwards - a bool 1/0 signifying forwards/backwards respectively

# Using A Path
start with your "path.txt" file
include it in your code like so
const std::string path = "/usd/path.txt";
make sure to include the file directory "/usd/" to the start

to actually use your path you need to include your files
pathingObj.DynamicPathingSystemRun(const std::string& path, lemlib::Chassis& chassis, float rpm, float wheelDiameter);

an example on how you would use it
pathingObj.DynamicPathingSystemRun(path, chassis, 360, 2.75);

# future updates
i am currently working on adding action support as this is just driving right now. your intake will have to wait for now.
