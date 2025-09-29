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
#define SANDBOX_HEIGHT 40
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
    {6, {TABLE_LENGTH + 780,  0}}
};
// No, x, y, score

struct Config {
    // config
    char* jaka_ip; // jaka mini ip
    int motor_com; // COM port for the motor
    char* repo_path;
    bool if_referee;

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
    const double correction_coefficient_y = 0.6, correction_coefficient_x = 1;
    const int deviser_x = 100, deviser_y = 200; // Deviser for adjacent learning, used to calculate the distance
    const double fiction_factor = 0.87; // Fiction factor that affects the velocity when ball falling
    const double check_bounce_threshold = 3;
} config;

//构建一个二维地图
struct GridCell {
    bool is_reachable = false;
    double actual_x = 0, actual_y = 0;
};

struct Point
{
    double x;
    double y;
    double t;  // time
    double vx; // velocity in x direction
    double vy; // velocity in y direction
};

Point ball_position;         // location of the ball
int balls_n;
Point hitting_position;
Point target;
char cmd = 'H'; // Command for the robot
std::mutex mtx;        // mutex for protecting shared resources
std::ofstream offset_file;  // 覆盖
RoboCupSSLClient client(10003, "224.5.23.1", "");
Point landing_position; // landing position of the ball
double score = 0;

JAKAZuRobot demo;
MotorIK mkmotor;

bool cell[8][25];

void get_vision();      // vision thread function
void get_referee();
void getBallLoc();
void load_reachability_map();

uint16_t combineBytes(uint8_t lowByte, uint8_t highByte) {
    return (highByte << 8) | lowByte;
}

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

void load_reachability_map() {
    std::ifstream file(string(config.repo_path) + "\\national\\static\\data_scan.csv");
    std::string line;
    std::getline(file, line); // header


    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 25; ++j) {
            cell[i][j] = true;
        }
    }

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string col_str, row_str, tx_str, ty_str, tz_str, ax_str, ay_str, az_str, status;
        std::getline(ss, col_str, ',');
        std::getline(ss, row_str, ',');
        std::getline(ss, tx_str, ',');
        std::getline(ss, ty_str, ',');
        std::getline(ss, tz_str, ',');
        std::getline(ss, ax_str, ',');
        std::getline(ss, ay_str, ',');
        std::getline(ss, az_str, ',');
        std::getline(ss, status);

        double tx, ty;
        tx = std::stod(tx_str);
        ty = std::stod(ty_str);

        int grid_x, grid_y;
        grid_x = (tx + 51) / 50;
        grid_y = (ty + 601) / 50;

        if (status != "Success") {
            cell[grid_x][grid_y] = false;
        }
    }

    file.close();
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

bool is_valid_offset(std::vector<Point>& full_track) {
    double half_table_length = TABLE_LENGTH / 2.0;

    size_t n = full_track.size();
    size_t cross_idx = n;
    for (size_t i = 0; i < n; ++i) {
        if (full_track[i].x > half_table_length) {
            cross_idx = i; break;
        }
    }

    if (cross_idx == n || cross_idx < 2 || (n - cross_idx) < 2) {
        return true;
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

    return angle_deg <= config.check_bounce_threshold;
};

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
    std::vector<Point> full_track;

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
        full_track.push_back(current_ball);

        // update the last ball position
        last_ball_pos = current_ball;
    }

    Sleep(2000);

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

    logger(Info, "Landing position: (", landing_position.x, ", ", landing_position.y, ").");
    outfile << 0 << "," << landing_position.x << "," << landing_position.y << std::endl;

    double offset_y = target.y - landing_position.y;
    double offset_x = target.x - landing_position.x;
    double offset = sqrt(pow(offset_x, 2) + pow(offset_y, 2));
    double add_score = (offset < 100.0) ? (3.0 * (1 - offset / 100.0)) : 0;
    score += add_score;

    logger(Info, "Add score: ", add_score, ". Now is ", score, ".");

    logger(Info, "Offset y: ", offset_y, "; offset x: ", offset_x);

    // if not valid offset, close file and return
    if (!is_valid_offset(full_track)) {
        logger(Info, "Invalid bounce detected. Offset will not be added.");
        outfile.close();
        if (config.if_record_trajectory) logger(Info, "Close file.");
        return;
    }

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
double define_motor_speed(double ball_x, double ball_y, double target_x, double target_y) {
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

    logger(Info, "Motor speed: ", motor_speed);

    return motor_speed;
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

bool hit_enumeration(double ball_x, double ball_y, CartesianPose& pos, JointValue& jval, JointValue& jvalH)
{
    for (int i = 5; i > 0; i--)
    {
        target = { std::get<0>(targets[i]), std::get<1>(targets[i]), 0, 0, 0 };

        logger(Info, "Trying target (", target.x, ", ", target.y, ").");

        double y_offset = (ball_y > 0) ? learning_left.get_offset_y(ball_x, ball_y) : learning_right.get_offset_y(ball_x, ball_y);
        double x_offset = (ball_y > 0) ? learning_left.get_offset_x(ball_x, ball_y) : learning_right.get_offset_x(ball_x, ball_y);

        Point modified_target = { target.x + x_offset, target.y + y_offset, 0, 0, 0 }; // Apply the offset to the target

        // calculate the angle to hit the ball
        double hitting_angle = calculate_hitting_angle(ball_position.x, ball_position.y, modified_target.x, modified_target.y);
        double hitting_angle_rad = hitting_angle * Deg2Rad;

        pos.tran.x = ball_x - config.paddle_ball_center_distance * cos(hitting_angle_rad);
        pos.tran.y = ball_y - config.paddle_ball_center_distance * sin(hitting_angle_rad);
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
            logger(Info, "Inverse succeeded for target (", target.x, ", ", target.y, ").");
            logger(Info, "Target after modified: (", modified_target.x, ", ", modified_target.y, "), Angle: ", hitting_angle, " degrees.");
            return true; // Exit the loop if inverse kinematics is successful
        }
        catch (std::runtime_error) {}
    }

    logger(Error, "Inverse error for all targets! Try to hit target 2!");

    //所有目标点的逆解都失败了，开始尝试微调球拍位置

    target = { std::get<0>(targets[2]), std::get<1>(targets[2]), 0, 0, 0 };

    logger(Info, "Trying target (", target.x, ", ", target.y, ").");

    double y_offset = (ball_y > 0) ? learning_left.get_offset_y(ball_x, ball_y) : learning_right.get_offset_y(ball_x, ball_y);
    double x_offset = (ball_y > 0) ? learning_left.get_offset_x(ball_x, ball_y) : learning_right.get_offset_x(ball_x, ball_y);

    Point modified_target = { target.x + x_offset, target.y + y_offset, 0, 0, 0 }; // Apply the offset to the target

    // calculate the angle to hit the ball
    double hitting_angle = calculate_hitting_angle(ball_position.x, ball_position.y, modified_target.x, modified_target.y);
    double hitting_angle_rad = hitting_angle * Deg2Rad;

    pos.tran.x = ball_x - config.paddle_ball_center_distance * cos(hitting_angle_rad);
    pos.tran.y = ball_y - config.paddle_ball_center_distance * sin(hitting_angle_rad);
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
    double adjust_length = 1;
    double adjust_round;
    if (ball_x <= 50) // 说明球在下边缘
        // 拍子落在设定位置的基础上，向击打方向微调，直到逆解成功，在此基础上打出
    {
        for (adjust_round = 1; adjust_round <= 10; adjust_round++)  // 每次尝试 1mm
        {
            pos.tran.y += adjust_length * sin(hitting_angle_rad);
            pos.tran.x += adjust_length * cos(hitting_angle_rad);
            try {
                demo.kine_inverse(&jval, &pos, &jvalH);
                logger(Info, "Inverse succeeded after adjusting +", adjust_length * adjust_round, " mm in hitting_direction.");
                return true;
            }
            catch (std::runtime_error) {}
        }
    }
    else if (ball_y >= 220) // 说明球在左边缘
    {
        for (adjust_round = 1; adjust_round <= 10; adjust_round++)
        {
            pos.tran.y += adjust_length * sin(hitting_angle_rad);
            pos.tran.x += adjust_length * cos(hitting_angle_rad);
            try {
                demo.kine_inverse(&jval, &pos, &jvalH);
                logger(Info, "Inverse succeeded after adjusting +", adjust_length * adjust_round, " mm in hitting_direction.");
                return true;
            }
            catch (std::runtime_error) {}
        }

    }
    else if (ball_y <= -220) //说明球在右边缘
    {
        for (adjust_round = 1; adjust_round <= 10; adjust_round++)
        {
            pos.tran.y += adjust_length * sin(hitting_angle_rad);
            pos.tran.x += adjust_length * cos(hitting_angle_rad);
            try {
                demo.kine_inverse(&jval, &pos, &jvalH);
                logger(Info, "Inverse succeeded after adjusting +", adjust_length * adjust_round, " mm in hitting_direction.");
                return true;
            }
            catch (std::runtime_error) {}
        }
    }
    else if (ball_x >= 20) //说明球在上边缘
    {
        for (adjust_round = 1; adjust_round <= 10; adjust_round++)
        {
            pos.tran.y -= adjust_length * sin(hitting_angle_rad);
            pos.tran.x -= adjust_length * cos(hitting_angle_rad);
            try {
                demo.kine_inverse(&jval, &pos, &jvalH);
                logger(Info, "Inverse succeeded after adjusting +", adjust_length * adjust_round, " mm in hitting_direction.");
                return true;
            }
            catch (std::runtime_error) {}
        }
    }
    else //球在中线
    {
        if (ball_y < 0) //球在右边
        {
            for (double adjust_round = 1; adjust_round <= 10; adjust_round++)
            {
                pos.tran.y -= adjust_length;
            }
            try {
                demo.kine_inverse(&jval, &pos, &jvalH);
                logger(Info, "Inverse succeeded after adjusting +", adjust_length * adjust_round, " mm in -y direction.");
                return true;
            }
            catch (std::runtime_error) {}
        }
        else //球在左边
        {
            for (double adjust_round = 1; adjust_round <= 10; adjust_round++)
            {
                pos.tran.y += adjust_length;
            }
            try {
                demo.kine_inverse(&jval, &pos, &jvalH);
                logger(Info, "Inverse succeeded after adjusting +", adjust_length * adjust_round, " mm in y direction.");
                return true;
            }
            catch (std::runtime_error) {}
        }
    }

    return false;
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

    load_reachability_map();
    logger(Info, "Map loading.")

    std::thread t2(getBallLoc);
    t2.detach(); // 分离线程，独立运行

    if (config.if_referee)
    {
        std::thread referee_thread(get_referee);
        referee_thread.detach();
        logger(Info, "Referee system connected.");
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
        double y_offset = (ball_y > 0) ? learning_left.get_offset_y(ball_x, ball_y) : learning_right.get_offset_y(ball_x, ball_y);
        double x_offset = (ball_y > 0) ? learning_left.get_offset_x(ball_x, ball_y) : learning_right.get_offset_x(ball_x, ball_y);
        logger(Info, "Choose target: (", target.x, ", ", target.y, ").");

        Point modified_target = { target.x + x_offset, target.y + y_offset, 0, 0, 0 }; // Apply the offset to the target
        logger(Info, "Target after offset: (", modified_target.x, ", ", modified_target.y, ").");

        // calculate the angle to hit the ball
        hitting_angle = calculate_hitting_angle(ball_position.x, ball_position.y, modified_target.x, modified_target.y);
        double hitting_angle_rad = hitting_angle * Deg2Rad;

        pos.tran.x = ball_x - config.paddle_ball_center_distance * cos(hitting_angle_rad);
        pos.tran.y = ball_y - config.paddle_ball_center_distance * sin(hitting_angle_rad);
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

        double ini_angle = -90;

        try {
            demo.kine_inverse(&jval, &pos, &jvalH);
        }
        catch (std::runtime_error) {
            logger(Error, "inverse error! Trying to enumerate all targets.");
            bool can = hit_enumeration(ball_x, ball_y, pos, jval, jvalH);
            if (!can) continue;
        }

        int grid_x = (int)((pos.tran.x + 51) / 50);
        int grid_y = (int)((pos.tran.y + 601) / 50);
		bool can_hit = cell[grid_x][grid_y];
        if (!can_hit) {
            logger(Error, "This position can't arrive! Trying to enumerate all targets.");
            bool can = hit_enumeration(ball_x, ball_y, pos, jval, jvalH);
            if (!can) continue;
        }

        logger(Move, "Jaka mini and motor preparing.");
        mkmotor.goToPos2(ini_angle, 1, 200, false);
        demo.joint_move(&jvalH, MoveMode::ABS, true, 1);

        Sleep(200);

        // Decide the speed based on the ball position
        double speed = define_motor_speed(ball_position.x, ball_position.y, modified_target.x, modified_target.y);

        logger(Info, "Motor hitting speed: ", speed, ".");

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

        logger(Info, "Round ", hit_round, " completed.");
        hit_round++;

        if (config.if_referee)
        {
            while (cmd == 'k') {}
        }
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