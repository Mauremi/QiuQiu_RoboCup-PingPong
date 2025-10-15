#pragma comment(lib, "winmm.lib")
#include <stdio.h>
#include "net/robocup_ssl_client.h"
#include "visionProto/messages_robocup_ssl_detection.pb.h"
#include "visionProto/messages_robocup_ssl_geometry.pb.h"
#include "visionProto/messages_robocup_ssl_wrapper.pb.h"

#include "gcProto/ssl_gc_referee_message.pb.h"

#include <iostream>
#include "endMotor/MotorIK.h"
#include <windows.h>
#include <time.h>
#include "2.2.7_fix_movel_acc_win/Windows/c&c++/inc_of_c++/jkerr.h"
#include "2.2.7_fix_movel_acc_win/Windows/c&c++/inc_of_c++/jktypes.h"
#include "2.2.7_fix_movel_acc_win/Windows/c&c++/inc_of_c++/JAKAZuRobot.h"
#include <chrono>

#include <thread>
#include <mutex>
#include <mmsystem.h>
#include <cmath>
#include <complex>
#include <conio.h> // Windows specific
#include <list>
#include <queue>
#include <vector>
#include <fstream>

using namespace std;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;


#define pi222 3.1415926
#define Deg2Rad (pi222 / 180)
#define Rad2Deg (180 / pi222)

#define TABLE_LENGTH 2740
#define TABLE_WIDTH 1525
#define SANDBOX_HEIGHT 100
#define GRAVITY 9796.7 // Gravity constant in mm/s^2

enum Logger { Info, Error, Move, Hint, Input };

namespace detail {
    template <typename... Args>
    inline void print_all(std::ostream& os, const Args&... args) {
        int _[] = { 0, ((void)(os << args), 0)... };
        (void)_;
    }
}

template <typename... Args>
void logger(Logger l, const Args&... args) {
    switch (l) {
    case Info:  std::cout << "\t[Info]  "; break;
    case Error: std::cout << "\t\033[0m\033[1;31m[Error] "; break;
    case Move:  std::cout << "\t\033[0m\033[1;32m[Move]  "; break;
    case Input: std::cout << "\t[Input] "; break;
    case Hint:  std::cout << "\t\033[0m\033[1;36m[Hint]  "; break;
    }

    detail::print_all(std::cout, args...);

    switch (l) {
    case Error: case Move: case Hint:
        std::cout << "\033[0m";
        break;
    default:
        break;
    }
    std::cout << '\n';
}


std::map<int, std::pair<double, double>> targets = {
    {1, {TABLE_LENGTH + 280,  560}},
    {2, {TABLE_LENGTH + 280,  0}},
    {3, {TABLE_LENGTH + 280, -560}},
    {4, {TABLE_LENGTH + 640,  250}},
    {5, {TABLE_LENGTH + 640, -250}},
    {6, {TABLE_LENGTH + 880,  0}}
};
// No, x, y, score

struct Config {
    // config
    char* jaka_ip; // jaka mini ip
    int motor_com; // COM port for the motor
    char* repo_path;
    bool if_referee;

    // speed
    const double velocity_factor = 31.5; // velocity shift factor 35.5 for second targets
    const double jaka_speed = 30;
    const int jaka_acceleration = 4000;

    // Motor speed 
    const double table_height_left = 760, table_height_right = 760;  // table height (left and right). If table is not flat, set the height of the left and right side of the table.
    double jaka_angle_left, jaka_angle_right; // Jaka mini angle for left and right side of the table

    // Paddle location
    const double paddle_z_left = 240, paddle_z_right = 240; // paddle height (when the ball is on the left or right side of the table)
    const double paddle_ball_center_hitting_distance = 10; // distance between paddle and ball center. Recommended value is 19 mm, which is the radius of the ball.
    const double paddle_ball_center_preparing_distance = 60;

    const double joint_threshold = 1;

    // Settings (IFs)
    const bool if_record_trajectory = true; // if record trajectory
    const bool if_vision_correction = false;

    // Adjacent learning
    const bool if_adjacent_learning = true; // if use adjacent learning
    const int offset_class = 1; // 1: , 2: Exponential
    const double correction_coefficient_y = 0.8, correction_coefficient_x = 0;
    const int deviser_x = 100, deviser_y = 200; // Deviser for adjacent learning, used to calculate the distance
    const double fiction_factor = 0.87; // Fiction factor that affects the velocity when ball falling

    const double check_bounce_threshold = 3;
    const double check_bounce_rmse = 7;
} config;


struct Point
{
    double x;
    double y;
    double t;  // time
    double vx; // velocity in x direction
    double vy; // velocity in y direction
};

struct Adjustment
{
    double ball_distance;
    double prepare_distance;
    double prepare_vertical;
    int ball_distance_x_sign;
    int ball_distance_y_sign;
    int prepare_vertical_x_sign;
    int prepare_vertical_y_sign;
};

Point ball_position;         // location of the ball
Point landing_position;
int balls_n;
Point hitting_position;
Point target;
char cmd = 'H'; // Command for the robot
std::mutex mtx;        // mutex for protecting shared resources
std::ofstream offset_file;  // 覆盖
JointValue jval;
double score = 0;

JAKAZuRobot demo;
MotorIK mkmotor;

void get_vision();      // vision thread function
void get_referee();
void getBallLoc();


class AdjacentLearning
{
    struct Offset
    {
        Point center; // Center of the offset
        double y_offset;
        double x_offset;
    };
    list<Offset> offsets_; // List of offsets for adjacent learning
public:
    AdjacentLearning() = default;
    ~AdjacentLearning() = default;

    // Add an offset for adjacent learning
    void add_offset(const double x, const double y, const double y_offset, const double x_offset) {
        if (abs(y_offset) >= 150 || abs(x_offset) >= 150)
        {
            logger(Error, "Invalid landing position! Offset will not be added.");
            return;
        }
        const Offset offset{ {x, y, 0, 0, 0}, y_offset, x_offset };
        offsets_.push_back(offset);
        // write csv
        if (offset_file.is_open()) {
            offset_file << x << ","
                << y << ","
                << x_offset << ","
                << y_offset << ","
                << "normal" << std::endl;
        }
    }

    // Get the offset for a specific position
    double get_offset_y(const double x, const double y) const {
        double offset_sum = 0.0;
        int index = 0;
        for (const auto& offset : offsets_) {
            index++;
            switch (config.offset_class)
            {
            case 1:
                offset_sum += config.correction_coefficient_y * offset.y_offset * exp(-sqrt(pow((x - offset.center.x) / config.deviser_x, 2) + pow((y - offset.center.y) / config.deviser_y, 2))); break;
            case 2: default:
                offset_sum += config.correction_coefficient_y * offset.y_offset * exp(-(pow((x - offset.center.x) / config.deviser_x, 2) - pow((y - offset.center.y) / config.deviser_y, 2))); break;
            }
        }
        logger(Info, index, " offsets.");
        return offset_sum;
    }

    //!!! to be implemented
    double get_offset_x(const double x, const double y) const
    {
        double offset_sum = 0.0;
        int index = 0;
        for (const auto& offset : offsets_) {
            index++;
            switch (config.offset_class)
            {
            case 1:
                offset_sum += config.correction_coefficient_x * offset.x_offset * exp(-sqrt(pow((x - offset.center.x) / config.deviser_x, 2) + pow((y - offset.center.y) / config.deviser_y, 2))); break;
            case 2: default:
                offset_sum += config.correction_coefficient_x * offset.x_offset * exp(-(pow((x - offset.center.x) / config.deviser_x, 2) - pow((y - offset.center.y) / config.deviser_y, 2))); break;
            }
        }
        return offset_sum;
    }
} learning_left, learning_right;

class LinearPredictor
{
    double slope;     // Slope of the line
    double intercept; // Y-intercept of the line
    // Y = slope * X + intercept
public:
    LinearPredictor(const std::vector<Point>& points)
    {
        // Based on series of points, calculate the slope and intercept for linear regression
        double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
        int n = points.size();

        for (const auto& point : points)
        {
            sum_x += point.x;
            sum_y += point.y;
            sum_xx += point.x * point.x;
            sum_xy += point.x * point.y;
        }

        // Calculate slope and y-intercept
        slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
        intercept = (sum_y - slope * sum_x) / n;
    }

    LinearPredictor(std::queue<Point> points)
    {
        // Based on series of points, calculate the slope and intercept for linear regression
        double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
        int n = points.size();

        while (points.size() > 0)
        {
            Point point = points.front();
            points.pop();
            sum_x += point.x;
            sum_y += point.y;
            sum_xx += point.x * point.x;
            sum_xy += point.x * point.y;
        }

        // Calculate slope and y-intercept
        slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
        intercept = (sum_y - slope * sum_x) / n;
    }

    double predict(double x)
    {
        // Predict y value for given x
        return slope * x + intercept;
    }
};

bool if_reach(double ball_x, double ball_y, JointValue& jvalH, Point& target) {
    double joint0 = jvalH.jVal[0] * 180 / pi222;
    double joint1 = jvalH.jVal[1] * 180 / pi222;
    double joint2 = jvalH.jVal[2] * 180 / pi222;
    double joint3 = jvalH.jVal[3] * 180 / pi222;
    double joint4 = jvalH.jVal[4] * 180 / pi222;
    double joint5 = jvalH.jVal[5] * 180 / pi222;
    logger(Info, joint0);
    logger(Info, joint1);
    logger(Info, joint2);
    logger(Info, joint3);
    logger(Info, joint4);
    logger(Info, joint5);
    if (joint0 == 0 && joint1 == 0 && joint2 == 0 && joint3 == 0 && joint4 == 0 && joint5 == 0)
    {
        logger(Error, "This position can't reach! try to hit target 2!");
        return false;
    }

    if (-360 * config.joint_threshold > joint0 || joint0 > 360 * config.joint_threshold)
    {
        logger(Error, "Joint 1 can't reach! try to hit target 2!");
        return false;
    }
    else if (-125 * config.joint_threshold > joint1 || joint1 > 125 * config.joint_threshold)
    {
        logger(Error, "Joint 2 can't reach! try to hit target 2!");
        return false;
    }
    else if (-130 * config.joint_threshold > joint2 || joint2 > 130 * config.joint_threshold)
    {
        logger(Error, "Joint 3 can't reach! try to hit target 2!");
        return false;
    }
    else if (-360 * config.joint_threshold > joint3 || joint3 > 360 * config.joint_threshold)
    {
        logger(Error, "Joint 4 can't reach! try to hit target 2!");
        return false;
    }
    else if (-120 * config.joint_threshold > joint4 || joint4 > 120 * config.joint_threshold)
    {
        logger(Error, "Joint 5 can't reach! try to hit target 2!");
        return false;
    }
    else if (-360 * config.joint_threshold > joint5 || joint5 > 360 * config.joint_threshold)
    {
        logger(Error, "Joint 6 can't reach! try to hit target 2!");
        return false;
    }
    else {

        return true;
    }
}

void strange_position(double ball_x, double ball_y, JointValue& jvalH, Point target)
{
    target = { Point{ targets[2].first, targets[2].second, 0, 0, 0 } };
    CartesianPose pos;
    pos.tran.x = ball_x - 18;
    pos.tran.y = ball_y > 0 ? ball_y - 130 : ball_y + 130;
    pos.tran.z = 95;
    pos.rpy.rx = 180 * Deg2Rad;
    pos.rpy.ry = 0 * Deg2Rad;
    pos.rpy.rz = 0 * Deg2Rad;

    try {
        demo.kine_inverse(&jval, &pos, &jvalH);
    }
    catch (std::runtime_error) { logger(Error, "Strange position is error!"); }

    bool can_reach = if_reach(ball_x, ball_y, jvalH, target);

    /*if (!can_reach)
         {
             pos.tran.y = ball_y + 80;
             try {
                 demo.kine_inverse(&jval, &pos, &jvalH);
             }
             catch (std::runtime_error) { logger(Error, "Strange position is error!"); }

             if (!if_reach(ball_x, ball_y, jvalH, target))
             {
                 logger(Error, "Strange position can't reach! Give up this ball!");
                 return;
             }
         }

     */
    JointValue jval_temp;
    jval_temp.jVal[0] = jval_temp.jVal[1] = jval_temp.jVal[2] = jval_temp.jVal[4] = jval_temp.jVal[5] = 0 * Deg2Rad;
    jval_temp.jVal[3] = 180 * Deg2Rad;

    mkmotor.goToPos2(ball_y > 0 ? -360 : 0, 1, 180, false);

    JointValue jval_temp1;
    jval_temp1.jVal[0] = jvalH.jVal[0];
    jval_temp1.jVal[1] = jvalH.jVal[1];
    jval_temp1.jVal[2] = jvalH.jVal[2];
    jval_temp1.jVal[3] = jvalH.jVal[3];
    jval_temp1.jVal[4] = 0;
    jval_temp1.jVal[5] = jvalH.jVal[5];

    //伸懒腰
    demo.joint_move(&jval_temp, ABS, true, 2);

    demo.joint_move(&jval_temp1, MoveMode::ABS, true, 1);
    Sleep(500);
    demo.joint_move(&jvalH, MoveMode::ABS, true, 1);
    Sleep(2000);
    mkmotor.goToPos2(-180, 1, 180, false);
    Sleep(1000);

    JointValue jvalH_hitting;

    jvalH_hitting.jVal[0] = jvalH.jVal[0];
    jvalH_hitting.jVal[1] = jvalH.jVal[1];
    jvalH_hitting.jVal[2] = jvalH.jVal[2];
    jvalH_hitting.jVal[3] = jvalH.jVal[3];
    jvalH_hitting.jVal[4] = jvalH.jVal[4];
    jvalH_hitting.jVal[5] = jvalH.jVal[5] + (ball_y > 0 ? 100.0 : -100.0) * pi222 / 180.0;

    Sleep(500);

    demo.joint_move(&jvalH_hitting, MoveMode::ABS, false, config.jaka_speed * pi222 / 180.0);

    Sleep(500);


    mkmotor.goToPos2(ball_y > 0 ? -80 : -300, ball_y > 0 ? 0 : 1, 730, false);

    jvalH_hitting.jVal[4] = 0;

    Sleep(1000);


    demo.joint_move(&jvalH_hitting, MoveMode::ABS, false, 1);
    demo.joint_move(&jval_temp, MoveMode::ABS, false, 1);
}


// calculate the angle to hit the ball
double calculate_hitting_angle(double ball_x, double ball_y, double posi_x, double posi_y)
{
    double run_x = posi_x - ball_x;
    double run_y = posi_y - ball_y;

    double angle_rad = atan2(run_y, run_x);
    double angle_deg = angle_rad * Rad2Deg;

    logger(Info, "Calculated angle: ", angle_deg, " degrees.");

    return angle_deg - (ball_y > 0 ? config.jaka_angle_left : config.jaka_angle_right);
}

double calculate_score(Point landing_position) {
    double distance;
    int index;
    for (int i = 1; i <= 6; i++) {
        double target_x = targets[i].first;
        double target_y = targets[i].second;
        if (i == 1 || distance >= sqrt(pow(landing_position.x - target_x, 2) + pow(landing_position.y - target_y, 2))) {
            distance = sqrt(pow(landing_position.x - target_x, 2) + pow(landing_position.y - target_y, 2));
            index = i;
        }
    }

    double full_score;
    double radius;
    switch (index)
    {
    case 1: case 3: full_score = 1.5; radius = 160; break;
    case 2: full_score = 1; radius = 200; break;
    case 4: case 5: default: full_score = 2.5; radius = 120; break;
    case 6: full_score = 3; radius = 100; break;
    }

    logger(Info, "Close to target ", index);

    double add_score = full_score * (1 - distance / radius);

    return add_score > 0 ? add_score : 0;
}

bool is_valid_offset(std::vector<Point>& full_track) {
    double half_table_length = TABLE_LENGTH / 2.0;

    size_t n = full_track.size();

    size_t cross_idx = n;
    for (size_t i = 0; i < n; ++i) {
        if (full_track[i].x > half_table_length) {
            cross_idx = i; break;
        }
    }

    Point p1 = full_track[cross_idx - 2];
    Point p2 = full_track[cross_idx - 1];
    double vx1 = p2.x - p1.x;
    double vy1 = p2.y - p1.y;

    Point q1 = full_track[cross_idx];
    Point q2 = full_track[cross_idx + 1];
    double vx2 = q2.x - q1.x;
    double vy2 = q2.y - q1.y;

    double dot = vx1 * vx2 + vy1 * vy2;
    double norm1 = sqrt(vx1 * vx1 + vy1 * vy1);
    double norm2 = sqrt(vx2 * vx2 + vy2 * vy2);
    if (norm1 < 1e-6 || norm2 < 1e-6) return true; // 向量过短，无法判定 => 认为有效

    double cos_theta = dot / (norm1 * norm2);
    if (cos_theta > 1.0) cos_theta = 1.0;
    else if (cos_theta < -1.0) cos_theta = -1.0;

    const double PI = std::acos(-1.0);
    double angle_deg = std::acos(cos_theta) * 180.0 / PI;

    logger(Info, "Bounce angle = ", angle_deg, " deg (threshold ", config.check_bounce_threshold, " deg)");

    if (angle_deg > config.check_bounce_threshold) {
        logger(Info, "Invalid offset: angle jump detected.");
        return false;
    }

    // linear regression
    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    for (const auto& pt : full_track) {
        sum_x += pt.x;
        sum_y += pt.y;
        sum_xy += pt.x * pt.y;
        sum_x2 += pt.x * pt.x;
    }
    double a = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x + 1e-8);
    double b = (sum_y - a * sum_x) / n;

    double mse = 0;
    for (const auto& pt : full_track) {
        double y_fit = a * pt.x + b;
        mse += (pt.y - y_fit) * (pt.y - y_fit);
    }
    mse /= n;
    double rmse = sqrt(mse);

    logger(Info, "Trajectory RMSE = ", rmse, " (threshold ", config.check_bounce_rmse, ")");

    if (rmse > config.check_bounce_rmse) {
        logger(Info, "Invalid offset: trajectory wave detected.");
        return false;
    }

    return true;
}

void record_trajectory() {
    // Open the CSV file for writing
    std::ofstream outfile(string(config.repo_path) + "\\national\\static\\data_trajectory.csv", std::ios::trunc);
    if (config.if_record_trajectory)
    {
        logger(Info, "CSV path: ", config.repo_path, "\\national\\static\\data_trajectory.csv");

        if (!outfile.is_open()) {
            logger(Error, "Can't open csv file!");
            return;
        }

        // write the header to the CSV file
        outfile << "time,x,y" << std::endl;
    }

    std::queue<Point> track;  // Vector to store the trajectory points

    Point last_ball_pos = { 0, 0, 0, 0, 0 };

    while (balls_n == 0);

    while (true) {
        std::lock_guard<std::mutex> lock(mtx);  // 使用 std::lock_guard 自动管理锁
        Point current_ball = ball_position;

        // Check if the ball is detected
        if (balls_n == 0) {
            Sleep(400);

            if (balls_n == 0)
            {
                logger(Info, "No ball detected. Recording stop.");
                break;  // No ball detected, stop recording
            }
            current_ball = ball_position;
        }

        // Check if the ball is within the table boundaries
        if (fabs(current_ball.x) > TABLE_LENGTH - 50 || fabs(current_ball.y) > TABLE_WIDTH - 30) {
            logger(Info, "Recording stop.");
            break;  // Automatically stop recording if the ball is out of bounds
        }

        // Check if the ball position has changed significantly
        if (fabs(current_ball.x - last_ball_pos.x) < 0.5 && fabs(current_ball.y - last_ball_pos.y) < 0.5) {
            continue;  // Skip writing if the position hasn't changed significantly
        }

        if (config.if_record_trajectory)
        {
            // write the current ball position and timestamp to the CSV file
            outfile << current_ball.t << "," << current_ball.x << "," << current_ball.y << std::endl;
        }

        track.push(current_ball); // Store the point in the track vector
        if (track.size() == 11)
        {
            track.pop();
        }

        // update the last ball position
        last_ball_pos = current_ball;
    }


    // Not enough points recorded, close the file and return
    if (track.size() <= 5) {
        logger(Error, "No sufficient points recorded.");
        outfile.close();  // close the file
        return;
    }
    LinearPredictor predictor(track); // Create a linear predictor based on the recorded trajectory

    double predicted_edge_y = predictor.predict(TABLE_LENGTH); // Predict the y value for the current x position

    if (config.if_adjacent_learning)
    {
        outfile << 0 << "," << TABLE_LENGTH << "," << predicted_edge_y << std::endl;
    }

    Sleep(3000);

    logger(Info, "Landing position: (", landing_position.x, ", ", landing_position.y, ").");
    outfile << 0 << "," << landing_position.x << "," << landing_position.y << std::endl;

    double offset_y = target.y - landing_position.y;
    double offset_x = target.x - landing_position.x;
    double add_score = calculate_score(landing_position);
    score += add_score;

    logger(Info, "Add score: ", add_score, ". Now is ", score, ".");

    logger(Info, "Offset y: ", offset_y, "; offset x: ", offset_x);



    if (hitting_position.y > 0) {
        learning_left.add_offset(hitting_position.x, hitting_position.y, offset_y, offset_x);
    }
    else {
        learning_right.add_offset(hitting_position.x, hitting_position.y, offset_y, offset_x);
    }

    outfile.close();  // close the file
    if (config.if_record_trajectory) logger(Info, "Close file.");
}

// Define the speed based on the ball position
double define_fake_motor_speed(double ball_x, double ball_y, double target_x, double target_y) {
    if (fabs(ball_x) < 0.01 && fabs(ball_y) < 0.01) {
        logger(Error, "No ball detected!");
        return 0; // No speed if no ball detected
    }

    // Calculate the speed based on the distance from the center
    double distance = sqrt(pow(target_x - ball_x, 2) + pow(target_y - ball_y, 2));
    double flight_distance = distance * (target_x - TABLE_LENGTH) / (target_x - ball_x);

    double off_table_y = ball_y + (target_y - ball_y) * (TABLE_LENGTH - ball_x) / (target_x - ball_x);
    double off_table_height = (TABLE_WIDTH / 2 + off_table_y) * (config.table_height_left - config.table_height_right) / TABLE_WIDTH + config.table_height_right - SANDBOX_HEIGHT;
    double flight_time = sqrt(off_table_height);

    double speed = flight_distance / flight_time;
    double motor_speed = speed * config.velocity_factor;

    return motor_speed;
}

bool adjust_paddle(double ball_x, double ball_y, CartesianPose& pos, JointValue& jval, JointValue& jvalH, Point& target)
{
    logger(Error, "333");
    double hitting_angle = calculate_hitting_angle(ball_position.x, ball_position.y, target.x, target.y);
    double hitting_angle_rad = hitting_angle * Deg2Rad;

    double adjust_length = 1;
    double adjust_round;
    if (ball_x <= 50) // 说明球在下边缘
        // 拍子落在设定位置的基础上，向击打方向微调，直到逆解成功，在此基础上打出
    {
        for (adjust_round = 1; adjust_round <= 20; adjust_round++)  // 每次尝试 1mm
        {
            pos.tran.x = ball_x - config.paddle_ball_center_hitting_distance * cos(hitting_angle_rad) + adjust_length * adjust_round * cos(hitting_angle_rad);
            pos.tran.y = ball_y - config.paddle_ball_center_hitting_distance * sin(hitting_angle_rad) + adjust_length * adjust_round * sin(hitting_angle_rad);
            pos.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;

            if (ball_y > 0) {
                pos.rpy.rx = (-90 + hitting_angle) * Deg2Rad;
                pos.rpy.ry = -90 * Deg2Rad;
                pos.rpy.rz = 0;
            }
            else {
                pos.rpy.rx = (90 - hitting_angle) * Deg2Rad;
                pos.rpy.ry = 90 * Deg2Rad;
                pos.rpy.rz = 0;
            }
            try {
                demo.kine_inverse(&jval, &pos, &jvalH);
            }
            catch (std::runtime_error) {}

            bool can_hit = if_reach(ball_x, ball_y, jvalH, target);
            if (can_hit) {
                logger(Info, "paddle move", adjust_length * adjust_round, "mm.");
                return true;
            }
        }
    }
    else if (-100 < ball_y < 100)//球在中线
    {
        if (ball_y < 0) //球在右边
        {
            for (double adjust_round = 1; adjust_round <= 12; adjust_round++)
            {

                pos.tran.x = ball_x - config.paddle_ball_center_hitting_distance * cos(hitting_angle_rad) + adjust_length * adjust_round * sin(hitting_angle_rad);
                pos.tran.y = ball_y - config.paddle_ball_center_hitting_distance * sin(hitting_angle_rad) - adjust_length * adjust_round * cos(hitting_angle_rad);
                pos.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;

                if (ball_y > 0) {
                    pos.rpy.rx = (-90 + hitting_angle) * Deg2Rad;
                    pos.rpy.ry = -90 * Deg2Rad;
                    pos.rpy.rz = 0;
                }
                else {
                    pos.rpy.rx = (90 - hitting_angle) * Deg2Rad;
                    pos.rpy.ry = 90 * Deg2Rad;
                    pos.rpy.rz = 0;
                }
                try {
                    demo.kine_inverse(&jval, &pos, &jvalH);
                }
                catch (std::runtime_error) {
                    logger(Error, "Inverse error!");
                }

                bool can_hit = if_reach(ball_x, ball_y, jvalH, target);

                if (can_hit) {
                    logger(Info, "paddle move", adjust_length * adjust_round, "mm.");
                    return true;
                }
                else
                {
                    logger(Error, "try once but error.");
                }

            }
        }
        else //球在左边
        {
            for (double adjust_round = 1; adjust_round <= 12; adjust_round++)
            {
                pos.tran.x = ball_x - config.paddle_ball_center_hitting_distance * cos(hitting_angle_rad) - adjust_length * adjust_round * sin(hitting_angle_rad);
                pos.tran.y = ball_y - config.paddle_ball_center_hitting_distance * sin(hitting_angle_rad) + adjust_length * adjust_round * cos(hitting_angle_rad);
                pos.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;

                if (ball_y > 0) {
                    pos.rpy.rx = (-90 + hitting_angle) * Deg2Rad;
                    pos.rpy.ry = -90 * Deg2Rad;
                    pos.rpy.rz = 0;
                }
                else {
                    pos.rpy.rx = (90 - hitting_angle) * Deg2Rad;
                    pos.rpy.ry = 90 * Deg2Rad;
                    pos.rpy.rz = 0;
                }
                try {
                    demo.kine_inverse(&jval, &pos, &jvalH);
                }
                catch (std::runtime_error) {}

                bool can_hit = if_reach(ball_x, ball_y, jvalH, target);
                if (can_hit) {
                    logger(Info, "paddle move", adjust_length * adjust_round, "mm.");
                    return true;
                }
            }
        }
    }
    else if (ball_y >= 220) // 说明球在左边缘
    {
        for (adjust_round = 1; adjust_round <= 20; adjust_round++)
        {
            pos.tran.x = ball_x - config.paddle_ball_center_hitting_distance * cos(hitting_angle_rad) + adjust_round * adjust_length * cos(hitting_angle_rad);
            pos.tran.y = ball_y - config.paddle_ball_center_hitting_distance * sin(hitting_angle_rad) + adjust_round * adjust_length * sin(hitting_angle_rad);
            pos.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;

            if (ball_y > 0) {
                pos.rpy.rx = (-90 + hitting_angle) * Deg2Rad;
                pos.rpy.ry = -90 * Deg2Rad;
                pos.rpy.rz = 0;
            }
            else {
                pos.rpy.rx = (90 - hitting_angle) * Deg2Rad;
                pos.rpy.ry = 90 * Deg2Rad;
                pos.rpy.rz = 0;
            }
            try {
                demo.kine_inverse(&jval, &pos, &jvalH);
            }
            catch (std::runtime_error) {}

            bool can_hit = if_reach(ball_x, ball_y, jvalH, target);
            if (can_hit) {
                logger(Info, "paddle move", adjust_length * adjust_round, "mm.");
                return true;
            }
        }

    }
    else if (ball_y <= -220) //说明球在右边缘
    {
        for (adjust_round = 1; adjust_round <= 20; adjust_round++)
        {
            pos.tran.x = ball_x - config.paddle_ball_center_hitting_distance * cos(hitting_angle_rad) + adjust_round * adjust_length * cos(hitting_angle_rad);
            pos.tran.y = ball_y - config.paddle_ball_center_hitting_distance * sin(hitting_angle_rad) + adjust_round * adjust_length * sin(hitting_angle_rad);
            pos.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;

            if (ball_y > 0) {
                pos.rpy.rx = (-90 + hitting_angle) * Deg2Rad;
                pos.rpy.ry = -90 * Deg2Rad;
                pos.rpy.rz = 0;
            }
            else {
                pos.rpy.rx = (90 - hitting_angle) * Deg2Rad;
                pos.rpy.ry = 90 * Deg2Rad;
                pos.rpy.rz = 0;
            }
            try {
                demo.kine_inverse(&jval, &pos, &jvalH);
            }
            catch (std::runtime_error) {}

            bool can_hit = if_reach(ball_x, ball_y, jvalH, target);
            if (can_hit) {
                logger(Info, "paddle move", adjust_length * adjust_round, "mm.");
                return true;
            }
        }
    }
    else if (ball_x >= 250) //说明球在上边缘
    {
        for (adjust_round = 1; adjust_round <= 20; adjust_round++)
        {
            pos.tran.x = ball_x - config.paddle_ball_center_hitting_distance * cos(hitting_angle_rad) + adjust_round * adjust_length * cos(hitting_angle_rad);
            pos.tran.y = ball_y - config.paddle_ball_center_hitting_distance * sin(hitting_angle_rad) - adjust_round * adjust_length * sin(hitting_angle_rad);
            pos.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;

            if (ball_y > 0) {
                pos.rpy.rx = (-90 + hitting_angle) * Deg2Rad;
                pos.rpy.ry = -90 * Deg2Rad;
                pos.rpy.rz = 0;
            }
            else {
                pos.rpy.rx = (90 - hitting_angle) * Deg2Rad;
                pos.rpy.ry = 90 * Deg2Rad;
                pos.rpy.rz = 0;
            }
            try {
                demo.kine_inverse(&jval, &pos, &jvalH);
            }
            catch (std::runtime_error) {}

            bool can_hit = if_reach(ball_x, ball_y, jvalH, target);
            if (can_hit) {
                logger(Info, "paddle move", adjust_length * adjust_round, "mm.");
                return true;
            }
        }
    }
    logger(Error, "adjust paddle error!");
    return false;
}




void joint_hit(int hitting_distance, double ball_x, double ball_y, double hitting_angle, double motor_speed, Point target)
{

    bool can_hit;

    //can_hit = inverse_ensure(hitting_distance, preparing_distance, adjust, motor_speed, ball_x, ball_y, hitting_angle);

    double hitting_angle_rad = hitting_angle * Deg2Rad;

    CartesianPose pos;
    pos.tran.x = ball_x - config.paddle_ball_center_hitting_distance * cos(hitting_angle_rad);
    pos.tran.y = ball_y - config.paddle_ball_center_hitting_distance * sin(hitting_angle_rad);
    pos.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;

    if (ball_y > 0) {
        pos.rpy.rx = (-90 + hitting_angle) * Deg2Rad;
        pos.rpy.ry = -90 * Deg2Rad;
        pos.rpy.rz = 0;
    }
    else {
        pos.rpy.rx = (90 - hitting_angle) * Deg2Rad;
        pos.rpy.ry = 90 * Deg2Rad;
        pos.rpy.rz = 0;
    }

    bool can_adjust = false;
    JointValue jvalH;
    try {
        demo.kine_inverse(&jval, &pos, &jvalH);
    }
    catch (std::runtime_error) {

        if (!can_adjust)
        {
            logger(Error, "Can't hit this target! Try to hit the other side!");
            target.y = target.y * (-1);

            double y_offset = ball_y > 0 ? learning_left.get_offset_y(ball_x, ball_y) : learning_right.get_offset_y(ball_x, ball_y);
            double x_offset = ball_y > 0 ? learning_left.get_offset_x(ball_x, ball_y) : learning_right.get_offset_x(ball_x, ball_y);
            logger(Info, "Choose target: (", target.x, ", ", target.y, ").");

            Point modified_target = { target.x + x_offset, target.y + y_offset, 0, 0, 0 }; // Apply the offset to the target
            logger(Info, "Target after offset: (", modified_target.x, ", ", modified_target.y, ").");

            // calculate the angle to hit the ball
            hitting_angle = calculate_hitting_angle(ball_position.x, ball_position.y, modified_target.x, modified_target.y);
            hitting_angle_rad = hitting_angle * Deg2Rad;

            pos.tran.x = ball_x - config.paddle_ball_center_hitting_distance * cos(hitting_angle_rad);
            pos.tran.y = ball_y - config.paddle_ball_center_hitting_distance * sin(hitting_angle_rad);
            pos.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;

            if (ball_y > 0) {
                pos.rpy.rx = (-90 + hitting_angle) * Deg2Rad;
                pos.rpy.ry = -90 * Deg2Rad;
                pos.rpy.rz = 0;
            }
            else {
                pos.rpy.rx = (90 - hitting_angle) * Deg2Rad;
                pos.rpy.ry = 90 * Deg2Rad;
                pos.rpy.rz = 0;
            }

            try {
                demo.kine_inverse(&jval, &pos, &jvalH);
            }
            catch (std::runtime_error) {
                can_adjust = adjust_paddle(ball_x, ball_y, pos, jval, jvalH, target);

            }
            if (!can_adjust)
            {
                logger(Error, "Can't inverse hitting position!");
            }
        }
    }

    bool can_reach = if_reach(ball_x, ball_y, jvalH, target);
    if (!can_reach) {
        logger(Info, "111");
        can_adjust = adjust_paddle(ball_x, ball_y, pos, jval, jvalH, target);
        logger(Error, can_adjust);
        if (!can_adjust && -100 < ball_y < 100) {
            logger(Info, "222");
            mtx.lock();
            ball_x = ball_position.x;
            ball_y = ball_position.y;
            mtx.unlock();
            logger(Error, "This position can't reach! Try to use strange position.");
            strange_position(ball_x, ball_y, jvalH, target);
        }
        return;
    }

    demo.joint_move(&jvalH, MoveMode::ABS, true, 1);

    JointValue jvalH_hitting;

    jvalH_hitting.jVal[0] = jvalH.jVal[0];
    jvalH_hitting.jVal[1] = jvalH.jVal[1];
    jvalH_hitting.jVal[2] = jvalH.jVal[2];
    jvalH_hitting.jVal[3] = jvalH.jVal[3];
    jvalH_hitting.jVal[4] = jvalH.jVal[4];
    jvalH_hitting.jVal[5] = jvalH.jVal[5] + ((ball_y > 0) ? -100.0 : 100.0) * pi222 / 180.0;

    Sleep(500);

    demo.joint_move(&jvalH_hitting, MoveMode::ABS, false, config.jaka_speed * pi222 / 180);

    Sleep(500);
    mkmotor.goToPos2((ball_y > 0) ? -270 : 90, 0, motor_speed, false); // !DIRECTION

}


void get_correction_data()
{
    ifstream angle_file(string(config.repo_path) + "\\national\\static\\data_AngleCorrection.csv");

    string line;
    getline(angle_file, line); // Skip the header line
    getline(angle_file, line); // Read the first line

    stringstream ss(line);
    double angle_left, angle_right;
    ss >> angle_left;
    ss.ignore(1, ','); // Ignore the comma
    ss >> angle_right;

    config.jaka_angle_left = angle_left;
    config.jaka_angle_right = angle_right;

    logger(Info, "Jaka mini paddle angle left: ", config.jaka_angle_left, " degrees, right: ", config.jaka_angle_right, " degrees.");
}

void read_config()
{
    ifstream config_file("config.csv");
    string line;

    getline(config_file, line); // Skip the header line
    getline(config_file, line); // Skip the header line

    stringstream ss(line);
    config.motor_com = 0;
    config.jaka_ip = new char[16];
    config.repo_path = new char[256];
    config.if_referee = false;
    ss >> config.motor_com;
    ss.ignore(1, ','); // Ignore the comma
    ss.getline(config.jaka_ip, 16, ',');
    ss.getline(config.repo_path, 256, ',');
    ss >> config.if_referee;
    // Log the configuration

    logger(Info, "Configuration loaded:",
        "Motor COM:", config.motor_com,
        "; Jaka IP:", config.jaka_ip,
        "; Repo Path:", config.repo_path,
        "; If Referee:", config.if_referee);
}

int main()
{
    read_config();
    offset_file.open(string(config.repo_path) + "\\national\\static\\data_offsets.csv", std::ios::trunc);
    offset_file << "x,y,x_offset,y_offset,point_type" << std::endl;

    logger(Info, "Main loop started.");

    // Separate thread for vision system
    std::thread t1(get_vision);
    t1.detach();
    logger(Info, "Vision system connected.");

    if (config.if_referee)
    {
        std::thread referee_thread(get_referee);
        referee_thread.detach();
        logger(Info, "Referee system connected.");

        std::thread t(getBallLoc);
        t.detach();
        logger(Info, "Sandbox system connected.");
    }

    demo.login_in(config.jaka_ip);
    logger(Info, "Jaka mini connected.");

    demo.power_on();
    demo.enable_robot();
    logger(Info, "Jaka mini empowered.");


    mkmotor.ID = 1;               // ID of the motor is 1
    mkmotor.reduceRatio = 10;     // Set the reduction ratio
    mkmotor.setSerial(config.motor_com, 115200); // Set the serial port COM number and baud rate
    mkmotor.start();
    logger(Info, "Motor connected.");

    JointValue jvalH;
    jval.jVal[0] = 0 * Deg2Rad;
    jval.jVal[1] = 0 * Deg2Rad;
    jval.jVal[2] = 90 * Deg2Rad;
    jval.jVal[3] = 0 * Deg2Rad;
    jval.jVal[4] = 0 * Deg2Rad;
    jval.jVal[5] = 0 * Deg2Rad;

    errno_t ret = ERR_SUCC;
    logger(Move, "Jaka mini initializing.");
    ret = demo.joint_move(&jval, MoveMode::ABS, true, 1);

    logger(Move, "Motor initializing.");
    double ini_dianji_degree = 0;
    mkmotor.goToPos2(ini_dianji_degree, 1, 200, false);

    int hit_round = 1;

    get_correction_data();

    while (cmd != 'e')
    {
        if (!config.if_referee)
        {
            logger(Hint, "Enter 'k' to begin hitting.");
            cin >> cmd;
        }

        if (cmd != 'k') continue;

        logger(Info, "Round ", hit_round, ".");

        logger(Info, "Begin hitting.");

        CartesianPose pos;
        double hitting_angle = 0;

        double ball_x, ball_y;
        mtx.lock();
        ball_x = ball_position.x;
        ball_y = ball_position.y;
        mtx.unlock();

        hitting_position = { ball_x, ball_y, 0, 0, 0 };

        if (fabs(ball_x) < 0.01 && fabs(ball_y) < 0.01)
        {
            logger(Error, "No ball detected!");
            continue;
        }

        logger(Info, "Ball position: (", ball_x, ", ", ball_y, ").");

        target = ball_y > 0 ? Point{ targets[5].first, targets[5].second, 0, 0, 0 } : Point{ targets[4].first, targets[4].second, 0, 0, 0 };
        double y_offset = ball_y > 0 ? learning_left.get_offset_y(ball_x, ball_y) : learning_right.get_offset_y(ball_x, ball_y);
        double x_offset = ball_y > 0 ? learning_left.get_offset_x(ball_x, ball_y) : learning_right.get_offset_x(ball_x, ball_y);
        logger(Info, "Choose target: (", target.x, ", ", target.y, ").");

        Point modified_target = { target.x + x_offset, target.y + y_offset, 0, 0, 0 }; // Apply the offset to the target
        logger(Info, "Target after offset: (", modified_target.x, ", ", modified_target.y, ").");

        // calculate the angle to hit the ball
        hitting_angle = calculate_hitting_angle(ball_position.x, ball_position.y, modified_target.x, modified_target.y);
        double hitting_angle_rad = hitting_angle * Deg2Rad;

        double ini_angle = -90;
        mkmotor.goToPos2(ini_angle, 1, 200, false);

        Sleep(200);

        // Decide the speed based on the ball position
        double speed = define_fake_motor_speed(ball_position.x, ball_position.y, modified_target.x, modified_target.y);  // A fake speed, containing jaka and motor
        logger(Info, "Fake speed: ", speed, " degrees/s.");
        speed -= (double)config.jaka_speed;
        logger(Info, "Motor speed: ", speed, " degrees/s.");

        joint_hit(config.paddle_ball_center_hitting_distance, ball_x, ball_y, hitting_angle, speed, modified_target);

        if (config.if_record_trajectory)
        {
            std::thread t2(record_trajectory);
            t2.detach();
        }

        Sleep(900);
        logger(Move, "Jaka mini and motor returning.");
        mkmotor.goToPos2(ini_dianji_degree, 1, 200, false);
        demo.joint_move(&jval, MoveMode::ABS, true, 7);

        logger(Info, "Round ", hit_round, " completed.");
        hit_round++;

        while (cmd == 'k');
    }
    return 0;
}


// Constantly get vision data in a separate thread
void get_vision()
{
    RoboCupSSLClient client;
    client.open(true);
    SSL_WrapperPacket packet;
    std::vector<Point> ballP;

    double a_x, b_x, c_x, a_y, b_y, c_y;

    if (config.if_vision_correction)
    {
        ifstream correction_file(string(config.repo_path) + "\\national\\public\data_VisionCorrection.csv");
        string line;
        getline(correction_file, line); // Skip the header line
        getline(correction_file, line); // Read the first line

        stringstream ss(line);

        ss >> a_x;
        ss.ignore(1, ','); // Ignore the comma
        ss >> b_x;
        ss.ignore(1, ','); // Ignore the comma
        ss >> c_x;
        ss.ignore(1, ','); // Ignore the comma
        ss >> a_y;
        ss.ignore(1, ','); // Ignore the comma
        ss >> b_y;
        ss.ignore(1, ','); // Ignore the comma
        ss >> c_y;
    }
    else
    {
        a_x = 1.0; b_x = 0.0; c_x = 0.0;
        a_y = 0.0; b_y = 1.0; c_y = 0.0;
    }

    while (true)
    {
        if (client.receive(packet))
        {
            if (packet.has_detection())
            {
                // Check if the packet contains detection data
                SSL_DetectionFrame detection = packet.detection();
                balls_n = detection.balls_size(); // Get the number of detected balls
                //std::cout << ball.x();
                // Update the ball position
                for (int i = 0; i < balls_n; i++)
                {
                    SSL_DetectionBall ball = detection.balls(i);
                    Point pp;
                    pp.x = a_x * ball.x() + b_x * ball.y() + c_x; // Apply correction
                    pp.y = a_y * ball.x() + b_y * ball.y() + c_y; // Apply correction
                    pp.t = detection.t_capture();
                    mtx.lock();
                    ball_position = pp;
                    mtx.unlock();

                    ballP.push_back(pp);
                }
            }
        }
    }
}

char translate_referee(Referee_Command command)
{

    switch (command)
    {
    case 0:
        cmd = 'H';
        break; // Halt
    case 1:
        cmd = 'S';
        break; // Stop
    case 2:
        cmd = ' ';
        break; // Normal start (Ready)
    case 8:
        cmd = 'k';
        break; // Kickoff Yellow
    case 9:
        cmd = 'K';
        break; // Kickoff Blue
    default:
        std::cout << "refereebox is fail !!!!! command : " << command << std::endl;
        cmd = 'H';
        break;
    }

    return cmd;
}

void get_referee()
{
    int round = 0;
    RoboCupSSLClient client(10003, "224.5.23.1", "");
    client.open(true); // 打开客户端连接
    Referee packet;
    byte datagram[20000];
    while (true)
    {
        if (client.receive(packet))
        {
            if (packet.has_command())
            { // 检查是否有命令
                Referee_Command next_command = packet.command();
                char cmd = translate_referee(next_command); // 将命令转换为控制字符
                // cout << cmd;
                if (cmd == 'k' && packet.has_yellow())
                {
                    // std::cout << "comand is " << cmd; // 输出命令
                    Referee_TeamInfo team = packet.yellow();
                    // std::cout << "current hit num is " << team.timeouts(); //输出当前的击打数
                    round = team.timeouts();
                }
                if (cmd == 'K' && packet.has_blue())
                {
                    // std::cout << "comand is " << cmd; // 输出命令
                    Referee_TeamInfo team = packet.blue();
                    // std::cout << "current hit num is " << team.timeouts();//输出当前的击打数
                }
            }
            if (packet.has_yellow())
            {
                Referee_TeamInfo team = packet.yellow();
            }
        }
    }
    return;
}

void getBallLoc()
{
    RoboCupSSLClient client(10005, "224.5.23.3", "");
    client.open(true); // 打开客户端连接
    ppBallPos packet;
    while (true) {
        if (client.receive(packet)) {
            if (packet.has_x() && packet.has_y()) { // 
                landing_position.x = packet.x() + 2740;
                landing_position.y = packet.y();
            }
        }
    }
    return;
}