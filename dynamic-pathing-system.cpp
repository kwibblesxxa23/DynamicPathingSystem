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

#define _USE_MATH_DEFINES//math for timeings

using namespace std;//using namespace for lazyness
using namespace DynamicPathingSystem;//using namespace for lazyness
vector<Pose> arrayList;//position array list
DynamicPathingSystem::Pathing pathingObjs;// the object initializer thing
//pose definition
Pose::Pose(double x, double y, double heading, bool forwards, int intake, bool mogoMech, bool doinker, bool redirect, int macros)
    : x(x), y(y), heading(heading), forwards(forwards), intake(intake), mogoMech(mogoMech), doinker(doinker), redirect(redirect), macros(macros) {}
//action items definition
ActionItems::ActionItems(pros::Motor intake, pros::Motor topIntake, pros::adi::DigitalOut mogoMech, pros::adi::DigitalOut doinker, pros::adi::DigitalOut redirect)
    : intake(intake), topIntake(topIntake), mogoMech(mogoMech), doinker(doinker), redirect(redirect) {}

//read file
void Pathing::readFile(const std::string& path){
    arrayList.clear();
    FILE* file = fopen(path.c_str(), "r");//opening the file
    if(file){//file exists check
        char buffer[120];//large enough buffer to read the file
        while(true){
            if(fgets(buffer, sizeof(buffer), file)){
                double x, y, h;
                int i, mac;
                bool b, m, d, r; //must be 1 or 0 for true and false in the path.txt
                std::stringstream(buffer) >> x >> y >> h >> b >> i >> m >> d >> r >> mac;
                Pose pose(x, y, h, b, i, m, d, r, mac);//make the pose
                arrayList.push_back(pose);//add the pose to the vector/arrayList
                //end of file processing
            }else{ break; }//exit condition
            pros::delay(10);//10ms delay to keep your robot happy
        }
    }
    fclose(file);//close the file to not kill your bot
}

// the main function of the Dynamic pathing system that does everything
void Pathing::DynamicPathingSystemRun(const std::string& path, lemlib::Chassis& chassis, float rpm, float wheelDiameter, ActionItems& actionItems){
    int size = arrayList.size();//get the size and set 0 as the starting number
    size--;
    Pose tempPose(arrayList[0]);// current pose
    Pose tempNextPose(arrayList[1]); //next pose
    chassis.setPose(tempPose.x, tempPose.y, tempPose.heading);//setting the starting poseition of the bot
    pros::delay(100);
    for(int i = 0; i <= size; i++){//go through the entire path and stop at the end
        tempPose = arrayList[i];//set the current pose
        if(i + 1 <= size){// null means end of path
            tempNextPose = arrayList[i+1];//set the next pose
        } else{ break; }// end the path when there isnt a next item

        double timeoutTime = 1000;//timeout time for lemlib - starts at 1000ms as its a useable time
        //conditions
        if(((tempPose.x != tempNextPose.x) || (tempPose.y != tempNextPose.y)) && (tempPose.heading != tempNextPose.heading)){
            //take distance using trig, figure out how fast the bot can move with stats from chassis, multiply the possible distance by the possible sieways movement, make sure the value is in ms
            if(abs(tempNextPose.heading - tempPose.heading) > 180){
                //cap math at 180
                timeoutTime = (sqrt(abs(pow(tempNextPose.y - tempPose.y, 2) + pow(tempNextPose.x - tempPose.x, 2))))/(((rpm * wheelDiameter) * M_PI)/60000) + (180 * M_PI);
            } else {
                //basic math to get an aproximate time
                timeoutTime = (sqrt(abs(pow(tempNextPose.y - tempPose.y, 2) + pow(tempNextPose.x - tempPose.x, 2))))/(((rpm * wheelDiameter) * M_PI)/60000) + (abs(tempNextPose.heading - tempPose.heading) * M_PI);
            }
            timeoutTime = timeoutTime * 2;//add in acceleration/deceleration for when its needed
            chassis.moveToPose(
                tempNextPose.x, //x
                tempNextPose.y, //y
                tempNextPose.heading, //theta
                timeoutTime, //timeout of 4000ms
                {.forwards = (bool)(tempNextPose.forwards)}
                //{.lead = 0.3, .horizontalDrift = 8}
            );

        } else if(((tempPose.x == tempNextPose.x) || (tempPose.y == tempNextPose.y)) && (tempPose.heading != tempNextPose.heading)){
            //figure out how fast the bot can move with stats from chassis, multiply distance by pi/4, make sure the value is in ms
            if(abs(tempNextPose.heading - tempPose.heading) > 180){//cap math at 180
                timeoutTime = (180 * M_PI)/(((rpm * wheelDiameter) * M_PI)/6000);
            } else {//basic math to get an aproximate time
                timeoutTime = (abs(tempNextPose.heading - tempPose.heading) * M_PI)/(((rpm * wheelDiameter) * M_PI)/6000);
            }
            timeoutTime = timeoutTime * 1.5;//add in acceleration/deceleration for when its needed
            chassis.turnToHeading(tempNextPose.heading, timeoutTime);//turn the bot using lemlib

        } else if(((tempPose.x != tempNextPose.x) || (tempPose.y != tempNextPose.y)) && (tempPose.heading == tempNextPose.heading)){
            // lemlib drive to point //
            //take distance using trig, figure out how fast the bot can move with stats from chassis, make sure the value is in ms
            timeoutTime = (sqrt(abs(pow(tempNextPose.y - tempPose.y, 2) + pow(tempNextPose.x - tempPose.x, 2))))/(((rpm * wheelDiameter) * M_PI)/60000);//timeout math explained below
            //distance/speed = time
            //distance = sqrt((x2-x1)^2 - (y2 - y1)^2)
            //units are in/min -> in/millisec min - > ms *60000
            timeoutTime = timeoutTime * 1.7;//add in acceleration/deceleration for when its needed
            chassis.moveToPoint(
                tempNextPose.x, //x
                tempNextPose.y, //y
                timeoutTime, //timeout of 4000ms
                {.forwards = tempNextPose.forwards}
            );//actually move
        } 
        // mogo mech checks - solenoids
        if(tempNextPose.mogoMech == 0){
            actionItems.mogoMech.set_value(false);
        } else if(tempNextPose.mogoMech == 1){
            actionItems.mogoMech.set_value(true);
        } else{}
    
        // doinker checks - solenoids
        if(tempNextPose.doinker == 0){
            actionItems.doinker.set_value(false);
        } else if(tempNextPose.doinker == 1){
            actionItems.doinker.set_value(true);
        } else{}
    
        // redirect checks - solenoids
        if(tempNextPose.redirect == 0){
            actionItems.redirect.set_value(false);
        } else if(tempNextPose.redirect == 1){
            actionItems.redirect.set_value(true);
        } else{}
            
        pros::Task task{[=] {
            int time = 0;
            time = timeoutTime;
            while(time >= 1){
                //redirect macros - can cancell out intake
                bool redirectCancel = 0;
                if(tempNextPose.macros == 1){
                    //check for a donut. if a donut hold the intake
                    actionItems.intake.brake();
                    actionItems.topIntake.brake();
                } else if(tempNextPose.macros == 2){
                    //slow the intake. load the donut into the intake
                    //watch for a donut, mainly when it leaves
                    //wait a small amount of time as we can calculate how long it takes a donut to move up the intake
                    //sensor updates every 10 ms - so we assume a tolerance of 10 ms
                    //intake spins at 600rpm with a 12 tooth sprocket
                    //donut travels 4 inches in 10ms
                } else{
                    if((tempNextPose.intake == 0)){
                        actionItems.intake.brake();
                        actionItems.topIntake.brake();
                    } else if(tempNextPose.intake == 1){
                        actionItems.intake.move_velocity(1000);
                        actionItems.topIntake.move_velocity(1000);
                    } else if(tempNextPose.intake == 2){
                        actionItems.intake.move_velocity(-1000);
                        actionItems.topIntake.move_velocity(-1000);
                    } else{}
                }
                // intake checks - we have 2 motor intake
                
                pros::delay(1);
                time--;
            }
    
        }};
        //fun debug stuff im going to use later
        //pros::lcd::print(2, "timeoutTime: %f\n", timeoutTime);
        pros::delay(timeoutTime);
        //pros::delay(1000);//change to above
    }
}

