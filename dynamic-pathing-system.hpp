#pragma once

#include "pros/motors.hpp"
#include "pros/misc.hpp"
#include "pros/adi.hpp"
#include "lemlib/api.hpp"
#include <string>

//using namespace std;
//load the path from a file

//pose condition
//pose(x, y, heading, );

//have the conditions (if statements) to determine what path to run

//if coords change, path
//if heading change, move to heading
//in both change, path to heading

namespace DynamicPathingSystem {

    class Pose {
        public:
        /**
         * @brief Construct a new Pose object
         *
         * @param x x pose
         * @param y y pose
         * @param heading heading
         * @param forwards forwards or back bool initialized in path file as a 1 or a 0
         * @param intake intake 0 = stop, 1 = backwards, 2 = forwards
         * @param mogoMech mogo mech?
         * @param doinker doinker?
         * @param redirect redirect?
         * @param macros redirect macros 0 = none, 1 = hold, 2 = load
         */
        Pose(double x, double y, double heading, bool forwards, int intake, bool mogoMech, bool doinker, bool redirect, int macros);
    
        double x;
        double y;
        double heading;
        bool forwards;
        int intake;
        bool mogoMech;
        bool doinker;
        bool redirect;
        int macros;
    };

    class ActionItems {
        public:
        /**
         * @brief Construct a new Pose object
         *
         * @param intake intake
         * @param topIntake other intake
         * @param mogoMech mogo mech
         * @param doinker doinker
         * @param redirect redirect
         */
         ActionItems(pros::Motor intake, pros::Motor topIntake, pros::adi::DigitalOut mogoMech, pros::adi::DigitalOut doinker, pros::adi::DigitalOut redirect);
    
         pros::Motor intake;
         pros::Motor topIntake;
         pros::adi::DigitalOut mogoMech;
         pros::adi::DigitalOut doinker;
         pros::adi::DigitalOut redirect;
    };

class Pathing {
    public:
    /**
     * @brief make the function to read the file
     *
     * @param path file path for the path
     *
     * @b Example
     * @code {.cpp}
     * // create the readFile function
     * std::string path = "path.txt";
     * void readFile(string path){
     * }
     * @endcode
     */
    void readFile(const std::string& path);

    public:
    /**
     * @brief make the function that actually drives your auton
     *
     * @param path file path for the path
     * @param chassis the bot to drive
     * @param rpm your bots rpm on the tires
     * @param wheelDiameter the wheel diameter your using
     * @param actionItems the action items class
     *
     * @b Example
     * @code {.cpp}
     * // create the DynamicPathingSystem function
     * std::string path = "path.txt";
     * lemlib::Chassis chassis(drivetrain, // drivetrain settings
     *                  lateral_controller, // lateral PID settings
     *                  angular_controller, // angular PID settings
     *                  sensors // odometry sensors
     * );
     * void DynamicPathingSystem(string path, lemlib::Chassis chassis){
     * }
     * @endcode
     */
    void DynamicPathingSystemRun(const std::string& path, lemlib::Chassis& chassis, float rpm, float wheelDiameter, ActionItems& actionItems);
};

} // end namespace


