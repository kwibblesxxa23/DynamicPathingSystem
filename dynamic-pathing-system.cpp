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
//pose
Pose::Pose(double xGoal, double y, double heading, bool forwards, int intake, bool mogoMech, bool doinker, bool redirect)
    : xGoal(xGoal), y(y), heading(heading), forwards(forwards), intake(intake), mogoMech(mogoMech), doinker(doinker), redirect(redirect) {}
//action items
ActionItems::ActionItems(pros::Motor intake, pros::Motor topIntake, pros::adi::DigitalOut mogoMech, pros::adi::DigitalOut doinker, pros::adi::DigitalOut redirect)
    : intake(intake), topIntake(topIntake), mogoMech(mogoMech), doinker(doinker), redirect(redirect) {}

//read file
void Pathing::readFile(const std::string& path){
    pros::lcd::print(1, "zoom zoom");
    FILE* file = fopen(path.c_str(), "r");
    if(file){
        pros::lcd::print(1, "file exists");
        char buffer[120];
        while(true){
            if(fgets(buffer, sizeof(buffer), file)){
                double x, y, h;
                int i;
                bool b, m, d, r; //must be 1 or 0 for true and false in the path.txt
                pros::lcd::print(2, "data: %f\n", buffer);
                //if (sizeof(buffer) < 5) { break; } // error
                std::stringstream(buffer) >> x >> y >> h >> b >> i >> m >> d >> r;
                Pose pose(x, y, h, b, i, m, d, r);//make the pose
                arrayList.push_back(pose);//add the pose to the vector/arrayList
                //end of file processing
            }else{
                break;
            }
            pros::delay(10);
        }
    }
    fclose(file);
}

void Pathing::RunActions(ActionItems& actionItems, int intake, bool mogoMech, bool doinker, bool redirect){
    //now make the checks for the variables and whats toggled or not

    // intake checks
    if(intake == 0){
        actionItems.intake.brake();
        actionItems.topIntake.brake();
    } else if(intake == 1){
        actionItems.intake.move_velocity(1000);
        actionItems.topIntake.move_velocity(1000);
    } else if(intake == 2){
        actionItems.intake.move_velocity(-1000);
        actionItems.topIntake.move_velocity(-1000);
    } else{
        actionItems.intake.brake();
        actionItems.topIntake.brake();
    }

    // mogo mech checks
    if(mogoMech == 0){
        actionItems.mogoMech.set_value(false);
    } else if(mogoMech == 1){
        actionItems.mogoMech.set_value(true);
    } else{}

    // doinker checks
    if(doinker == 0){
        actionItems.doinker.set_value(false);
    } else if(doinker == 1){
        actionItems.doinker.set_value(true);
    } else{}

    // redirect checks
    if(redirect == 0){
        actionItems.redirect.set_value(false);
    } else if(redirect == 1){
        actionItems.redirect.set_value(true);
    } else{}
}

void Pathing::DynamicPathingSystemRun(const std::string& path, lemlib::Chassis& chassis, float rpm, float wheelDiameter, ActionItems& actionItems){
    pros::lcd::print(1, "readfile");
    pathingObjs.readFile(path);
    int size = arrayList.size() - 1;
    Pose tempPose(0.0, 0.0, 0.0, true, 0, 0, 0, 0);      
    Pose tempNextPose(0.0, 0.0, 0.0, true, 0, 0, 0, 0);
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
            pros::lcd::print(1, "path");
        } else{ break; }// null means end of path

        //calculate the timeout time to be the fastest possible
        double timeoutTime = 10;

        //conditions
        if(((tempX == nextX) || (tempY == nextY)) && (tempHeading != nextHeading)){
            
            //lemlib turn to heading code
            //take distance using trig like a controller by going a distance of 6 inches away from the center, then trig line between that using pythagorean therum, 
            //figure out how fast the bot can move with stats from chassis, multiply distance by pi/4, make sure the value is in ms
            double error = nextHeading - tempHeading;
            //normalise the heading
            if(abs(error) > 180){
                //cap math at 180
                timeoutTime = (180 * M_PI)/(((rpm * wheelDiameter) * M_PI)/6000);
            } else {
                //basic math to get an aproximate time
                timeoutTime = (abs(error) * M_PI)/(((rpm * wheelDiameter) * M_PI)/6000);
            }
            //SPEED MULTIPLYER BASED OFF OF STATS
            //add in acceleration/deceleration for when its needed
            //timeoutTime = timeoutTime + 100;
            chassis.turnToHeading(nextHeading, timeoutTime);//turn the bot
           // pros::delay(timeoutTime);
            pros::lcd::print(1, "heading");
        } else if(((tempX != nextX) || (tempY != nextY)) && (tempHeading == nextHeading)){
            
            //lemlib drive to point
            //take distance using trig, figure out how fast the bot can move with stats from chassis, make sure the value is in ms
            timeoutTime = (sqrt(abs(pow(nextY - tempY, 2) + pow(nextX - tempX, 2))))/(((rpm * wheelDiameter) * M_PI)/60000);//distance/speed = time
            //distance = sqrt((x2-x1)^2 - (y2 - y1)^2)
            //units are in/min -> in/millisec min - > ms *60000

            //SPEED MULTIPLYER BASED OFF OF STATS
            //add in acceleration/deceleration for when its needed
            //timeoutTime = timeoutTime + 100;
            chassis.moveToPoint(
                nextX, //x
                nextY, //y
                timeoutTime, //timeout of 4000ms
                {.forwards = tempNextPose.forwards}
            );
            //pros::delay(timeoutTime);
            pros::lcd::print(1, "straight");
        } else if(((tempX != nextX) || (tempY != nextY)) && (tempHeading == nextHeading)){
            
            double error = nextHeading - tempHeading;
            //lemlib swing to point code with heading
            //take distance using trig, figure out how fast the bot can move with stats from chassis, multiply the possible distance by the possible sieways movement, make sure the value is in ms
            if(abs(error) > 180){
                //cap math at 180
                timeoutTime = (sqrt(abs(pow(nextY - tempY, 2) + pow(nextX - tempX, 2))))/(((rpm * wheelDiameter) * M_PI)/60000) + (180 * M_PI);
            } else {
                //basic math to get an aproximate time
                timeoutTime = (sqrt(abs(pow(nextY - tempY, 2) + pow(nextX - tempX, 2))))/(((rpm * wheelDiameter) * M_PI)/60000) + (abs(error) * M_PI);
            }

            //SPEED MULTIPLYER BASED OFF OF STATS
            //add in acceleration/deceleration for when its needed
            //timeoutTime = timeoutTime + 100;
            chassis.moveToPose(
                nextX, //x
                nextY, //y
                nextHeading, //theta
                timeoutTime, //timeout of 4000ms
                {.forwards = (bool)(tempNextPose.forwards)}
                //{.lead = 0.3, .horizontalDrift = 8}
            );
            pros::lcd::print(1, "both");
            //pros::delay(timeoutTime);
        }
        // run the actions after the math to make sure we hit them
        pathingObjs.RunActions(actionItems, tempPose.intake, tempPose.mogoMech, tempPose.doinker, tempPose.redirect);

        pros::lcd::print(3, "cooking");
        pros::lcd::print(2, "timeoutTime: %f\n", timeoutTime);
        //pros::delay(timeoutTime);
        pros::delay(1000);//change to above
    }
}

