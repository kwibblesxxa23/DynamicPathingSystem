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
         * @param xGoal x pose
         * @param y y pose
         * @param heading heading
         * @param forwards forwards or back bool initialized in path file as a 1 or a 0
         * @param intake intake?
         * @param mogoMech mogo mech?
         * @param doinker doinker?
         * @param redirect redirect?
         */
        Pose(double xGoal, double y, double heading, bool forwards, int intake, bool mogoMech, bool doinker, bool redirect);
    
        double xGoal;
        double y;
        double heading;
        bool forwards;
        int intake;
        bool mogoMech;
        bool doinker;
        bool redirect;
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

    public:
    /**
     * @brief make the function that actually drives your auton
     *
     * @param path file path for the path
     * @param chassis the bot to drive
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
    void RunActions(ActionItems& actionItems, int intake, bool mogoMech, bool doinker, bool redirect);
};

} // end namespace


