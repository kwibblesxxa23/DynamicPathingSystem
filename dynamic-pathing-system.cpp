//file loading system
//line one is the size of the path, how long to read to
//the rest are pose points

//file processing system
//pose filling out
//objects into arrays

#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <math.h>
#include <cmath>
#include "pros/motors.hpp"
#include "pros/misc.hpp"
#include "pros/adi.hpp"
#include "lemlib/api.hpp"
#include "dynamic-pathing-system.hpp"

#define _USE_MATH_DEFINES

using namespace std;
using namespace DynamicPathingSystem;
vector<Pose> arrayList;
DynamicPathingSystem::Pathing pathingObjs;
Pose::Pose(double xGoal, double y, double heading, bool forwards)
    : xGoal(xGoal), y(y), heading(heading), forwards(forwards) {}

void Pathing::readFile(const std::string& path){
    FILE* file = fopen(path.c_str(), "r");
    if(file){
        char buffer[50];
        while(true){
            if(fgets(buffer, sizeof(buffer), file)){
                double x, y, h;
                bool b; //must be 1 or 0 for true and false in the path.txt
                if (!(std::stringstream(buffer) >> x >> y >> h >> b)) { break; } // error
                std::stringstream(buffer) >> x >> y >> h >> b;
                Pose pose(x, y, h, b);//make the pose
                arrayList.push_back(pose);//add the pose to the vector/arrayList
                //end of file processing
            }
        }
    }
    fclose(file);
}

void Pathing::DynamicPathingSystemRun(const std::string& path, lemlib::Chassis& chassis, float rpm, float wheelDiameter){
    pathingObjs.readFile(path);
    int size = arrayList.size() - 1;
    Pose tempPose(0.0, 0.0, 0.0, true);      
    Pose tempNextPose(0.0, 0.0, 0.0, true);
    double tempX, tempY, tempHeading;
    double nextX, nextY, nextHeading;
    tempPose;// = arrayList[i];
    chassis.setPose(tempPose.xGoal, tempPose.y, tempPose.heading);
    for(int i = 0; i <= size; i++){
        tempPose = arrayList[i];
        tempX = tempPose.xGoal;
        tempY = tempPose.y;
        tempHeading = tempPose.heading;
        if(i + 1 < size){// null means end of path
            tempNextPose = arrayList[i+1];
            nextX = tempNextPose.xGoal;
            nextY = tempNextPose.y;
            nextHeading = tempNextPose.heading;
        } else{ break; }// null means end of path

        //calculate the timeout time to be the fastest possible
        double timeoutTime;

        //conditions
        if(((tempX == nextX) || (tempY == nextY)) && (tempHeading != nextHeading)){
            //lemlib turn to heading code
            //take distance using trig like a controller by going a distance of 6 inches away from the center, then trig line between that using pythagorean therum, 
            //figure out how fast the bot can move with stats from chassis, multiply distance by pi/4, make sure the value is in ms
            double error = nextHeading - tempHeading;
            //normalise the heading
            if(abs(error) > 180){
                //cap math at 180
                timeoutTime = 180 * M_PI;
            } else {
                //basic math to get an aproximate time
                timeoutTime = abs(error) * M_PI;
            }
            //SPEED MULTIPLYER BASED OFF OF STATS
            timeoutTime = timeoutTime/((rpm * wheelDiameter) * M_PI);
            

            chassis.turnToHeading(nextHeading, timeoutTime);//turn the bot
        } else if(((tempX != nextX) || (tempY != nextY)) && (tempHeading == nextHeading)){
            //lemlib drive to point
            //take distance using trig, figure out how fast the bot can move with stats from chassis, make sure the value is in ms
            timeoutTime = sqrt(pow(abs(nextY), 2) + pow(abs(tempX), 2));

            //SPEED MULTIPLYER BASED OFF OF STATS
            timeoutTime = timeoutTime/((rpm * wheelDiameter) * M_PI);

            chassis.moveToPoint(
                nextX, //x
                nextY, //y
                timeoutTime, //timeout of 4000ms
                {.forwards = tempNextPose.forwards}
            );
        } else if(((tempX != nextX) || (tempY != nextY)) && (tempHeading == nextHeading)){
            double error = nextHeading - tempHeading;
            //lemlib swing to point code with heading
            //take distance using trig, figure out how fast the bot can move with stats from chassis, multiply the possible distance by the possible sieways movement, make sure the value is in ms
            if(abs(error) > 180){
                //cap math at 180
                timeoutTime = sqrt(pow(abs(nextY), 2) + pow(abs(tempX), 2)) + (180 * M_PI);
            } else {
                //basic math to get an aproximate time
                timeoutTime = sqrt(pow(abs(nextY), 2) + pow(abs(tempX), 2)) + (abs(error) * M_PI);
            }

            //SPEED MULTIPLYER BASED OFF OF STATS
            timeoutTime = timeoutTime/((rpm * wheelDiameter) * M_PI);
            
            chassis.moveToPose(
                nextX, //x
                nextY, //y
                nextHeading, //theta
                timeoutTime, //timeout of 4000ms
                {.forwards = (bool)(tempNextPose.forwards)}
                //{.lead = 0.3, .horizontalDrift = 8}
            );
        }
        pros::delay(20);
    }
}

