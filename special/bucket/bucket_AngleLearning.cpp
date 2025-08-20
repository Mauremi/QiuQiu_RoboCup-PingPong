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
#pragma comment(lib, "winmm.lib")

#include "SDK2.1.7/Windows/c&c++/inc_of_c++/jkerr.h"
#include "SDK2.1.7/Windows/c&c++/inc_of_c++/jktypes.h"
#include "SDK2.1.7/Windows/c&c++/inc_of_c++/JAKAZuRobot.h"
#include <chrono>
#include <thread>
#include <mutex>
#define pi222 3.1415926
#define Deg2Rad (pi222 / 180)
#define Rad2Deg (180 / pi222)
#include <cmath>
#include <vector>
#include <fstream>
#include <complex>
#include <conio.h> // Windows specific

#include <mmsystem.h>
#include <queue>

using namespace std;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

enum Logger { Info, Error, Move, Hint, Input };
void logger(Logger l, const char* content = "") {
    switch (l) {
    case Info:
        std::cout << "\t[Info]  " << content; break;
    case Error:
        printf("\t\033[0m\033[1;31m[Error] %s\033[0m", content); break;
    case Move:
        printf("\t\033[0m\033[1;32m[Move]  %s\033[0m", content); break;
    case Input:
        std::cout << "\t[Input] " << content; break;
    case Hint:
        printf("\t\033[0m\033[1;36m[Hint]  %s\033[0m", content); break;
    }

    if (content != "") {
        std::cout << std::endl;
    }
}

template <class T>
T std_in(T& variable, const char* hint = "") {
    if (hint != "") {
        logger(Hint, hint);
    }

    std::cin >> variable;
    logger(Input, variable);

    return variable;
}

#define TABLE_LENGTH 2740
#define TABLE_WIDTH 1525
#define BUCKET_HEIGHT 100
#define VISUAL_BOARDER 762.5 // Visual border for the ball position
#define GRAVITY 9796.7 // Gravity constant in mm/s^2

std::map<int, std::tuple<double, double, int>> buckets = {
    {1, {TABLE_LENGTH + 280,  0,   1}},
    {2, {TABLE_LENGTH + 280,  560, 2}},
    {3, {TABLE_LENGTH + 280, -560, 2}},
    {4, {TABLE_LENGTH + 540,  250, 3}},
    {5, {TABLE_LENGTH + 540, -250, 3}}//,
    //{6, {TABLE_LENGTH + 780,  0,   4}}
};
// No, x, y, score

struct Config {
    // Config
    const char* jaka_ip = "192.168.1.100"; // jaka mini ip
    int motor_com = 10; // COM of motor
    const char* csv_path = "C:\\Robot\\2025PingPongCampus\\codes\\special\\bucket_trajectory.csv"; // CSV file path to record trajectory

    // Motor speed
    const double table_height_left = 760, table_height_right = 760;  // table height (left and right). If table is not flat, set the height of the left and right side of the table.
    const double velocity_factor = 34.5; // velocity shift factor 35.5 for second targets
    double jaka_angle_left, jaka_angle_right; // Jaka mini angle for left and right side of the table

    // Paddle location
    const double paddle_z_left = 240, paddle_z_right = 240; // paddle height (when the ball is on the left or right side of the table)
    const double paddle_ball_center_distance = 10; // distance between paddle and ball center. Recommended value is 19 mm, which is the radius of the ball.

    // Settings (IFs)
    const bool if_record_trajectory = true; // if record trajectory
    const bool if_vision_correction = false;

    // Adjacent learning
    const bool if_adjacent_learning = true; // if use adjacent learning
    const int offset_class = 1; // 1: , 2: Exponential
    const double correction_coefficient = 0.8;
    const int deviser_x = 100, deviser_y = 200; // Deviser for adjacent learning, used to calculate the distance
    const double fiction_factor = 0.87; // Fiction factor that affects the velocity when ball falling
} config;

struct Point
{
    double x;
    double y;
    double t;  // time
    double vx; // velocity in x direction
    double vy; // velocity in y direction
};

class AdjacentLearning
{
    struct Offset
    {
        Point center; // Center of the offset
        double angle_offset;
    };
    list<Offset> offsets_; // List of offsets for adjacent learning
public:
    AdjacentLearning() = default;
    ~AdjacentLearning() = default;

    // Add an offset for adjacent learning
    void add_offset(const double x, const double y, const double angle_offset) {
        const Offset offset{ {x, y, 0, 0, 0}, angle_offset };
        offsets_.push_back(offset);
    }

    // Get the offset for a specific position
    double get_offset(const double x, const double y) const {
        double offset_sum = 0.0;
        int index = 0;
        for (const auto& offset : offsets_) {
            index++;
            switch (config.offset_class)
            {
            case 1:
                offset_sum += config.correction_coefficient * offset.angle_offset * exp(-sqrt(pow((x - offset.center.x) / config.deviser_x, 2) + pow((y - offset.center.y) / config.deviser_y, 2))); break;
            case 2: default:
                offset_sum += config.correction_coefficient * offset.angle_offset * exp(-(pow((x - offset.center.x) / config.deviser_x, 2) - pow((y - offset.center.y) / config.deviser_y, 2))); break;
            }
        }
        logger(Info);
        std::cout << index << " offsets.\n";
        return offset_sum;
    }
} learning;

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

Point ball_position;         // location of the ball
int balls_n;
Point hitting_position;
Point target;
double goA;
char cmd = 'H'; // Command for the robot

std::mutex mtx;        // mutex for protecting shared resources

void getVision();      // vision thread function
void getReferee();    // referee thread function

// calculate the angle to hit the ball
double goAvalue(double ball_x, double ball_y, double posi_x, double posi_y)
{
    double run_x = posi_x - ball_x;
    double run_y = posi_y - ball_y;

    double angle_rad = atan2(run_y, run_x);
    double angle_deg = angle_rad * Rad2Deg;

    logger(Info);
    std::cout << "Calculated angle: " << angle_deg << " degrees." << std::endl;

    return angle_deg - (ball_y > 0 ? config.jaka_angle_left : config.jaka_angle_right);
}

// to be implemented
// choose the best target based on the ball position
Point choice(double ball_x, double ball_y) {
    int best_bucket = -1;
    int best_score = -1;
    double min_center_dist = 1e9;
    Point target;

    logger(Info, "Start choosing a target.");

    for (std::map<int, std::tuple<double, double, int>>::const_iterator it = buckets.begin(); it != buckets.end(); ++it) {
        // All targets
        int id = it->first;
        double gx = std::get<0>(it->second);
        double gy = std::get<1>(it->second);
        int score = std::get<2>(it->second);

        logger(Info);
        std::cout << "Considering target (" << gx << ", " << gy << ").\n";

        bool prefer_direction = (ball_y > 0 && gy <= 0) || (ball_y < 0 && gy >= 0);

        if (prefer_direction) {
            logger(Info, "Good direction.");
            if (score > best_score || (score == best_score && fabs(gy) < min_center_dist)) {
                best_score = score;
                min_center_dist = fabs(gy);
                best_bucket = id;
                target.x = gx;
                target.y = gy;
                logger(Info);
                std::cout << "Target changed. Target ID " << best_bucket << ", Position(" << target.x << ", " << target.y << "), Centre distance minimum " << min_center_dist << " mm.\n";
            }
            else {
                logger(Info, "Not a good bucket.");
            }
        }
        else {
            logger(Info, "Not in good direction. Not a good bucket.");
        }
    }

    if (best_bucket == -1) {
        logger(Info, "No goal fits requirement. Lower the limit.");
        for (std::map<int, std::tuple<double, double, int>>::const_iterator it = buckets.begin(); it != buckets.end(); ++it) {/////////////////
            // All targets
            int id = it->first;
            double gx = std::get<0>(it->second);
            double gy = std::get<1>(it->second);
            int score = std::get<2>(it->second);

            logger(Info);
            std::cout << "Considering target (" << gx << ", " << gy << ").\n";

            if (score > best_score || (score == best_score && fabs(gy) < min_center_dist)) {
                best_score = score;
                min_center_dist = fabs(gy);
                best_bucket = id;
                target.x = gx;
                target.y = gy;
                logger(Info);
                std::cout << "Target changed. Target ID " << best_bucket << ", Position(" << target.x << ", " << target.y << "), Centre distance minimum " << min_center_dist << " mm.\n";
            }
            else
            {
                logger(Info, "Not a good goal.");
            }
        }
    }

    if (best_bucket == -1) {  // No target found, default to bucket 6
        logger(Error, "Not valid goal. Default: Goal 6.");
        best_bucket = 6;
        target.x = std::get<0>(buckets[best_bucket]);
        target.y = std::get<1>(buckets[best_bucket]);
    }

    return target;
}

void record_trajectory() {
    // Open the CSV file for writing
    std::ofstream outfile(config.csv_path, std::ios::trunc);
    if (config.if_record_trajectory)
    {
        logger(Info);
        std::cout << "CSV path: " << config.csv_path << "\n";

        if (!outfile.is_open()) {
            logger(Error, "Can't open csv file!");
            return;
        }

        // write the header to the CSV file
        outfile << "time,x,y" << std::endl;
    }

    queue<Point> track;  // Vector to store the trajectory points

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

    // predict the landing position
    double velocity_x = config.fiction_factor * (track.back().x - track.front().x) / (track.back().t - track.front().t);
    double height = (TABLE_WIDTH / 2 + predicted_edge_y) * (config.table_height_left - config.table_height_right) / TABLE_WIDTH + config.table_height_right - BUCKET_HEIGHT;
    double flight_time = sqrt(2.0 * height / GRAVITY);
    double predicted_x = TABLE_LENGTH + velocity_x * flight_time; // Predict the x value based on the velocity and flight time
    double predicted_y = predictor.predict(predicted_x);

    double height_ground = (TABLE_WIDTH / 2 + predicted_edge_y) * (config.table_height_left - config.table_height_right) / TABLE_WIDTH + config.table_height_right;
    double flight_time_ground = sqrt(2.0 * height_ground / GRAVITY);
    double predicted_ground_x = TABLE_LENGTH + velocity_x * flight_time_ground; // Predict the x value based on the velocity and flight time to ground
    double predicted_ground_y = predictor.predict(predicted_ground_x);

    if (config.if_adjacent_learning)
    {
        outfile << 0 << "," << TABLE_LENGTH << "," << predicted_edge_y << std::endl;
        outfile << 0 << "," << predicted_x << "," << predicted_y << std::endl;
        outfile << 0 << "," << predicted_ground_x << "," << predicted_ground_y << std::endl;
    }

    double delta_angle = goA - goAvalue(hitting_position.x, hitting_position.y, predicted_x, predicted_y);

    learning.add_offset(hitting_position.x, hitting_position.y, delta_angle);

    if (config.if_adjacent_learning)
    {
        logger(Info);
        std::cout << "Predicted point: (" << predicted_x << ", " << predicted_y << "). Offset: " << delta_angle << " degrees. ";
        std::cout << "Offset added for adjacent learning.\n";
    }

    outfile.close();  // close the file
    if (config.if_record_trajectory) logger(Info, "Close file.");
}

// Motor control and Jaka mini control class instances
JAKAZuRobot demo;
MotorIK mkmotor;

// Define the speed based on the ball position
double define_motor_speed(double ball_x, double ball_y, double target_x, double target_y) {
    if (fabs(ball_x) < 0.01 && fabs(ball_y) < 0.01) {
        logger(Error, "No ball detected!");
        return 0; // No speed if no ball detected
    }

    // Calculate the speed based on the distance from the center
    double distance = sqrt(pow(target_x - ball_x, 2) + pow(target_y - ball_y, 2));
    double flight_distance = distance * (target_x - TABLE_LENGTH) / (target_x - ball_x);

    double off_table_y = ball_y + (target_y - ball_y) * (TABLE_LENGTH - ball_x) / (target_x - ball_x);
    double off_table_height = (TABLE_WIDTH / 2 + off_table_y) * (config.table_height_left - config.table_height_right) / TABLE_WIDTH + config.table_height_right - BUCKET_HEIGHT;
    double flight_time = sqrt(off_table_height);

    double speed = flight_distance / flight_time;
    double motor_speed = speed * config.velocity_factor;

    return motor_speed;
}

void get_correction_data()
{
    ifstream angle_file("C:\\Robot\\2025PingPongCampus\\codes\\special\\BucketAngle_correction.csv");

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

    logger(Info);
    std::cout << "Jaka mini paddle angle left: " << config.jaka_angle_left << " degrees, right: " << config.jaka_angle_right << " degrees." << std::endl;
}

bool hit_enumeration(double ball_x, double ball_y, CartesianPose& pos, JointValue& jval, JointValue& jvalH)
{
    for (int i = 5; i > 0; i--)
    {
        target = { std::get<0>(buckets[i]), std::get<1>(buckets[i]), 0, 0, 0 };

        logger(Info);
        std::cout << "Trying target (" << target.x << ", " << target.y << ").\n";

        // calculate the angle to hit the ball
        double goA = goAvalue(ball_position.x, ball_position.y, target.x, target.y);
        double goA_rad = goA * Deg2Rad;

        pos.tran.x = ball_x - config.paddle_ball_center_distance * cos(goA_rad);
        pos.tran.y = ball_y - config.paddle_ball_center_distance * sin(goA_rad);
        pos.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;

        if (ball_y > 0) {
            pos.rpy.rx = (-90 + goA) * Deg2Rad;
            pos.rpy.ry = -90 * Deg2Rad;
            pos.rpy.rz = 0;
        }
        else {
            pos.rpy.rx = (90 - goA) * Deg2Rad;
            pos.rpy.ry = 90 * Deg2Rad;
            pos.rpy.rz = 0;
        }

        try {
            demo.kine_inverse(&jval, &pos, &jvalH);
            logger(Info);
            std::cout << "Inverse succeeded for target (" << target.x << ", " << target.y << ").\n";
            return true; // Exit the loop if inverse kinematics is successful
        }
        catch (std::runtime_error) {}
    }

    // Trying the hardest target
    {
        target = { TABLE_LENGTH + 780, 0, 0, 0, 0 };
        logger(Info);
        std::cout << "Trying target (" << target.x << ", " << target.y << ").\n";

        // calculate the angle to hit the ball
        double goA = goAvalue(ball_position.x, ball_position.y, target.x, target.y);
        double goA_rad = goA * Deg2Rad;

        pos.tran.x = ball_x - config.paddle_ball_center_distance * cos(goA_rad);
        pos.tran.y = ball_y - config.paddle_ball_center_distance * sin(goA_rad);
        pos.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;

        if (ball_y > 0) {
            pos.rpy.rx = (-90 + goA) * Deg2Rad;
            pos.rpy.ry = -90 * Deg2Rad;
            pos.rpy.rz = 0;
        }
        else {
            pos.rpy.rx = (90 - goA) * Deg2Rad;
            pos.rpy.ry = 90 * Deg2Rad;
            pos.rpy.rz = 0;
        }

        try {
            demo.kine_inverse(&jval, &pos, &jvalH);
            logger(Info);
            std::cout << "Inverse succeeded for target (" << target.x << ", " << target.y << ").\n";
            return true; // Exit the loop if inverse kinematics is successful
        }
        catch (std::runtime_error) {}
    }

    logger(Error, "Inverse error for all targets!");
    return false;
}

int main()
{
    ifstream com_file("C:\\Robot\\2025PingPongCampus\\codes\\special\\com.csv");
    string line;

    getline(com_file, line); // Skip the header line

    stringstream ss(line);
    ss >> config.motor_com;

    logger(Info, "Main loop started.");

    std::thread referee_thread(getReferee);
    referee_thread.detach();
    logger(Info, "Referee system connected.");

    // Separate thread for vision system
    std::thread t1(getVision);
    t1.detach();
    logger(Info, "Vision system connected.");

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

    JointValue jval, jvalH;
    jval.jVal[0] = 0 * Deg2Rad;
    jval.jVal[1] = 0 * Deg2Rad;
    jval.jVal[2] = 90 * Deg2Rad;
    jval.jVal[3] = 0 * Deg2Rad;
    jval.jVal[4] = 0 * Deg2Rad;
    jval.jVal[5] = 0 * Deg2Rad;

    errno_t ret = ERR_SUCC;
    logger(Move, "Jaka mini initializing.");
    ret = demo.joint_move(&jval, MoveMode::ABS, true, 5);

    logger(Move, "Motor initializing.");
    double ini_dianji_degree = 0;
    mkmotor.goToPos2(ini_dianji_degree, 1, 200, false);

    int hit_round = 1;

    get_correction_data();

    while (cmd != 'e')
    {
		if (cmd != 'k') continue;

        logger(Info);
        std::cout << "Round " << hit_round << ".\n";

        logger(Info, "Begin hitting.");

        CartesianPose pos;

        double ball_x, ball_y;
        mtx.lock();
        ball_x = ball_position.x;
        ball_y = ball_position.y;
        mtx.unlock();

        if (fabs(ball_x) < 0.01 && fabs(ball_y) < 0.01)
        {
            logger(Error, "No ball detected!");
            continue;
        }

        logger(Info);
        std::cout << "Ball position: (" << ball_x << ", " << ball_y << ").\n";

        target = choice(ball_x, ball_y);
        logger(Info);
        std::cout << "Choose target: (" << target.x << ", " << target.y << ").\n";

        // calculate the angle to hit the ball
        goA = goAvalue(ball_position.x, ball_position.y, target.x, target.y) - learning.get_offset(ball_position.x, ball_position.y);
        double goA_rad = goA * Deg2Rad;

        pos.tran.x = ball_x - config.paddle_ball_center_distance * cos(goA_rad);
        pos.tran.y = ball_y - config.paddle_ball_center_distance * sin(goA_rad);
        pos.tran.z = ball_y > 0 ? config.paddle_z_left : config.paddle_z_right;

        if (ball_y > 0) {
            pos.rpy.rx = (-90 + goA) * Deg2Rad;
            pos.rpy.ry = -90 * Deg2Rad;
            pos.rpy.rz = 0;
        }
        else {
            pos.rpy.rx = (90 - goA) * Deg2Rad;
            pos.rpy.ry = 90 * Deg2Rad;
            pos.rpy.rz = 0;
        }

        double ini_angle = -90;

        try {
            demo.kine_inverse(&jval, &pos, &jvalH);
        }
        catch (std::runtime_error) {
            logger(Error, "inverse error! Trying to enumerate all targets.");
            bool can = hit_enumeration(ball_x, ball_y, pos, jval, jvalH);
            if (!can) continue;
        }

        logger(Move, "Jaka mini and motor preparing.");
        mkmotor.goToPos2(ini_angle, 1, 200, false);
        demo.joint_move(&jvalH, MoveMode::ABS, true, 1);

        Sleep(200);

        double speed;

        // Decide the speed based on the ball position
        if (target.x == TABLE_LENGTH + 280)
            speed = 470;
        else
            speed = define_motor_speed(ball_position.x, ball_position.y, target.x, target.y);

        logger(Info);
        std::cout << "Motor hitting speed: " << speed << ".\n";

        logger(Move, "Motor hitting.");
        mkmotor.goToPos2((ball_y > 0) ? (ini_angle - 180) : (ini_angle + 180), 0, speed, false); // !DIRECTION

        if (config.if_record_trajectory)
        {
            std::thread t2(record_trajectory);
            t2.detach();
        }

        Sleep(900);
        logger(Move, "Jaka mini and motor returning.");
        mkmotor.goToPos2(ini_dianji_degree, 1, 200, false);
        demo.joint_move(&jval, MoveMode::ABS, true, 7);

        logger(Info);
        std::cout << "Round " << hit_round << " completed.\n";
        hit_round++;
    }
    return 0;
}

// Constantly get vision data in a separate thread
void getVision()
{
    RoboCupSSLClient client;
    client.open(true);
    SSL_WrapperPacket packet;
    std::vector<Point> ballP;

    double a_x, b_x, c_x, a_y, b_y, c_y;

    if (config.if_vision_correction)
    {
        ifstream correction_file("C:\\Robot\\2025PingPongCampus\\codes\\special\\visual_correction.csv");
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

char translateRefMsgs(Referee_Command command)
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
// 函数：持续接收裁判信息
void getReferee()
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
                char cmd = translateRefMsgs(next_command); // 将命令转换为控制字符
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
                // std::cout << "current score is " << team.score();//输出得分
            }
        }
    }
    return;
}