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
#include "SDK2.1.7/Windows/c&c++/inc_of_c++/jkerr.h"
#include "SDK2.1.7/Windows/c&c++/inc_of_c++/jktypes.h"
#include "SDK2.1.7/Windows/c&c++/inc_of_c++/JAKAZuRobot.h"
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
#include <Eigen/Dense>

using namespace std;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;


#define pi222 3.1415926
#define Deg2Rad (pi222 / 180)
#define Rad2Deg (180 / pi222)

#define TABLE_LENGTH 2740
#define TABLE_WIDTH 1525
#define GOAL_WIDTH 80
#define GOAL_THICKNESS 20
#define HALF_TABLE_LENGTH 1370


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


std::map<int, std::pair<double, double>> goals = {
    {1, {TABLE_LENGTH - GOAL_THICKNESS,  725}},
    {2, {TABLE_LENGTH - GOAL_THICKNESS,  540}},
    {3, {TABLE_LENGTH - GOAL_THICKNESS,  360}},
    {4, {TABLE_LENGTH - GOAL_THICKNESS,  180}},
    {5, {TABLE_LENGTH - GOAL_THICKNESS,    0}},
    {6, {TABLE_LENGTH - GOAL_THICKNESS, -180}},
    {7, {TABLE_LENGTH - GOAL_THICKNESS, -360}},
    {8, {TABLE_LENGTH - GOAL_THICKNESS, -540}},
    {9, {TABLE_LENGTH - GOAL_THICKNESS, -725}},
};
// No, x, y, score

struct Config {
    // config
    char* jaka_ip; // jaka mini ip
    int motor_com; // COM port for the motor
    char* repo_path;
    bool if_referee;

    const int goal_choice = 2;

    // motor speed
    const int speed = 740; // Maximum speed for the motor
    const int hitting_threshold = 250; // Threshold ball_x to decide when to hit the ball
    const int jaka_speed = 5;

    // paddle position
    const double paddle_z_left = 240, paddle_z_right = 240; // Paddle heights for left and right sides
    const double paddle_ball_center_distance = 10; // Distance from paddle center to ball center
    const double jaka_offset_x = 0, jaka_offset_y = -10;
    double jaka_angle_left, jaka_angle_right;

    // trajectory prediction
    const double linear_weight_quadratic = 0.9; // Weight for linear prediction in trajectory prediction
    const int quadratic_prediction_mode = 1; // Choose among 0(batch), 1(whole), 2(half)

    // IFs
    const bool if_record_trajectory = true;
    const bool if_record_prediction = true;
    const bool if_vision_correction = false;

    // Equation
    const double parameter_e_left = 0.74, parameter_e_right = 0.76;
    const double velocity_y_parameter = 5 / 7.0; // Coefficient for velocity_y in the equation
} config;


struct Point
{
    double x;
    double y;
    double t; // time
};

template<typename T, size_t Capacity>
class SafeRingBuffer {
    std::deque<T> buffer;
    std::mutex mtx;

public:
    void push(const T& value) {
        std::lock_guard<std::mutex> lock(mtx);
        if (buffer.size() >= Capacity)
            buffer.pop_front();
        buffer.push_back(value);
    }

    std::vector<T> snapshot() {
        std::lock_guard<std::mutex> lock(mtx);
        return { buffer.begin(), buffer.end() };
    }

    bool empty() {
        std::lock_guard<std::mutex> lock(mtx);
        return buffer.empty();
    }

    T back() {
        std::lock_guard<std::mutex> lock(mtx);
        return buffer.back();
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(mtx);
        buffer.clear();
    }
};

template<typename T>
class SafeValue
{
private:
    T data;
    mutable std::mutex mtx; // mutable 允许 const 函数加锁

public:
    SafeValue() : data{} {}  // 默认初始化
    SafeValue(const T& value) : data(value) {}

    // 设置整个值
    void set(const T& value) {
        std::lock_guard<std::mutex> lock(mtx);
        data = value;
    }

    // 获取副本
    T get() const {
        std::lock_guard<std::mutex> lock(mtx);
        return data;
    }
};

std::ofstream trajectory_outfile;
std::ofstream prediction_outfile;
SafeRingBuffer<Point, 100> half_table_points;
SafeRingBuffer<Point, 100> full_table_points;
SafeValue<int> balls_n;
SafeValue<bool> is_hitting(false);
SafeValue<char> cmd('H');
JointValue jval, jvalH, inijval;

SafeRingBuffer<Point, 15> latest_ball_positions;
SafeValue<Point> ball_position;
SafeValue<double> hitting_angle;
SafeValue<double> linear_predicted_y;
SafeValue<double> quadratic_predicted_y;
SafeValue<double> coming_angle;
SafeValue<double> coming_velocity;

bool go_to_left = true; // Flag to determine if the paddle should go to the left or right

JAKAZuRobot demo;
MotorIK mkmotor;

void get_vision();   // Function to get vision data
void get_referee();  // Function to get referee data


// Linear Predictors for trajectory prediction Class
/*
LinearPredictor(vector<Point>& points): Initializes with a series of points (vector<Point>), and calculates the slope and intercept for linear regression.
predict(double x): Calculates the predicted y value for a given x using the linear equation Y = slope * X + intercept.
*/
class LinearPredictor
{
    double intercept; // Y-intercept of the line
    // Y = slope * X + intercept
public:
    double slope;     // Slope of the line

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

    LinearPredictor(std::queue<Point>& points)
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

// Quadratic Predictors for trajectory prediction Class
/*
QuadraticPredictor(vector<Point>& points): Initializes with a series of points (vector<Point>), and calculates the coefficients for quadratic regression.
predict(double x): Calculates the predicted y value for a given x using the quadratic equation Y = ax^2 + bx + c.
*/
class QuadraticPredictor
{
    double a;       // Coefficient for x^2
    double b_coeff; // Coefficient for x
    double c;       // Constant term
public:
    QuadraticPredictor(const std::vector<Point> points)
    {
        int n = points.size();

        // Constructing the matrices
        Eigen::MatrixXd A(n, 3); // Design matrix
        Eigen::VectorXd b(n);    // Observations vector

        for (int i = 0; i < n; ++i)
        {
            A(i, 0) = points[i].x * points[i].x; // x^2
            A(i, 1) = points[i].x;               // x
            A(i, 2) = 1;                         // Constant term
            b(i) = points[i].y;                  // y
        }

        // Solve for coefficients using the least squares solution
        // A * coeffs = b => coeffs = (A^T * A)^(-1) * A^T * b
        Eigen::VectorXd coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * b);

        // Coefficients for y = ax^2 + bx + c
        a = coeffs(0);
        b_coeff = coeffs(1);
        c = coeffs(2);
    }

    double predict(double x)
    {
        // Predict y value for given x using the quadratic formula
        return a * x * x + b_coeff * x + c;
    }
};

// Get the current time in microseconds since epoch
uint64_t get_current_time_microseconds()
{
    FILETIME ft;
    GetSystemTimeAsFileTime(&ft); // Get the current system time in FILETIME format

    ULARGE_INTEGER ui;
    ui.LowPart = ft.dwLowDateTime;
    ui.HighPart = ft.dwHighDateTime;

    uint64_t microseconds = (ui.QuadPart - 116444736000000000ULL) / 10;
    return microseconds;
}

double get_distance(double ball_x, double ball_y, double posi_x, double posi_y)
{
    double sqr = (ball_x - posi_x) * (ball_x - posi_x) + (ball_y - posi_y) * (ball_y - posi_y);
    return sqrt(sqr);
}

// Get the direction of the ball based on its trajectory
double get_ball_coming_direction(std::vector<Point> balls)
{
    if (balls.size() < 2)
    {
        return 0.0;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xy = 0.0;
    double sum_xx = 0.0;

    int n = balls.size();

    for (const auto& point : balls)
    {
        sum_x += point.x;
        sum_y += point.y;
        sum_xy += point.x * point.y;
        sum_xx += point.x * point.x;
    }

    // m is the slope of the line, that is the tangent of the angle
    double m = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);

    double angle_radians = std::atan(m);
    double angle_degrees = angle_radians * Rad2Deg;
    return angle_degrees;
}

// Check if the ball is moving toward the robot
// !!! Maybe about to modify
bool is_ball_moving_toward_robot()  // modified
{
    std::vector<Point> track = latest_ball_positions.snapshot();

    if (track[0].x <= 80 || track[0].x >= 2740) {
        return false;
    }

    double directionSum = 0;
    int validCount = 0;

    for (int i = 1; i < track.size(); i++) {
        double dx = track[i].x - track[i - 1].x;
        directionSum += dx;
        validCount++;
    }

    double averageDirection = directionSum / validCount;

    return averageDirection < -5.0;
}

void record_trajectory_after() {
    queue<Point> track;  // Vector to store the trajectory points

    Point last_ball_pos = { 0, 0, 0 };

    while (true) {
        double ball_x, ball_y, time_stamp;

        Point ball_position_copy = ball_position.get();
        ball_x = ball_position_copy.x;
        ball_y = ball_position_copy.y;
        time_stamp = ball_position_copy.t;

        // Check if the ball is detected
        if (balls_n.get() == 0) {
            Sleep(400);

            if (balls_n.get() == 0)
            {
                logger(Info, "No ball detected. Recording stop.");
                break;  // No ball detected, stop recording
            }
            ball_x = ball_position_copy.x;
            ball_y = ball_position_copy.y;
            time_stamp = ball_position_copy.t;
        }

        // Check if the ball is within the table boundaries
        if (fabs(ball_x) > TABLE_LENGTH - 150 || fabs(ball_y) > TABLE_WIDTH - 30) {
            logger(Info, "Recording stop.");
            break;  // Automatically stop recording if the ball is out of bounds
        }

        // Check if the ball position has changed significantly
        if (fabs(ball_x - last_ball_pos.x) < 0.5 && fabs(ball_y - last_ball_pos.y) < 0.5) {
            continue;  // Skip writing if the position hasn't changed significantly
        }

        if (config.if_record_trajectory)
        {
            // write the current ball position and timestamp to the CSV file
            trajectory_outfile << time_stamp << "," << ball_x << "," << ball_y << std::endl;
        }

        track.push({ ball_x, ball_y, time_stamp }); // Store the point in the track vector

        if (track.size() == 11)
        {
            track.pop();
        }

        // update the last ball position
        last_ball_pos.x = ball_x;
        last_ball_pos.y = ball_y;
        last_ball_pos.t = time_stamp;
    }

    LinearPredictor predictor(track); // Create a linear predictor based on the recorded trajectory

    double predicted_y = predictor.predict(TABLE_LENGTH - GOAL_THICKNESS); // Predict the y value for the current x position
    double offset_y = (predicted_y > 0) ? 725 : -725;
    offset_y -= predicted_y; // Calculate the offset based on the predicted y value

    trajectory_outfile << 0 << "," << TABLE_LENGTH - GOAL_THICKNESS << "," << predicted_y << std::endl;

    trajectory_outfile.close();  // close the file
}

class Equation
{
    double coming_angle_, coming_velocity_, paddle_velocity_, leaving_angle_;
    double paddle_angle_;
    double parameter_e_;

    // f(\phi)
    double equation(double phi, double theta, double theta_prime, double v, double v_p, double e) {
        double delta_phi_theta = phi - theta;           // φ - θ
        double delta_theta_prime_phi = theta_prime - phi; // θ' - φ

        // avoid division by zero
        if (std::abs(std::sin(delta_theta_prime_phi)) < 1e-10) {
            return (delta_theta_prime_phi > 0) ? 1e10 : -1e10;
        }

        // left part of the equation
        double left_part = config.velocity_y_parameter * v * std::sin(delta_phi_theta) * std::cos(delta_theta_prime_phi);

        // right part of the equation
        double right_part = e * v * std::cos(delta_phi_theta) * std::sin(delta_theta_prime_phi) + (1.0 + e) * v_p * std::sin(delta_theta_prime_phi);

        return left_part - right_part;
    }

    // Newton's method to solve the equation
    double solve(double theta, double theta_prime, double v, double v_p, double e, double initial_guess) {
        const double tolerance = 1e-10;
        const double h = 1e-6;

        double phi = initial_guess;
        while (true) {
            double f = equation(phi, theta, theta_prime, v, v_p, e);
            if (std::abs(f) < tolerance) return phi; // 收敛检查

            double f_plus = equation(phi + h, theta, theta_prime, v, v_p, e);
            double f_minus = equation(phi - h, theta, theta_prime, v, v_p, e);
            double df_dphi = (f_plus - f_minus) / (2 * h);



            double phi_new = phi - f / df_dphi;

            if (phi_new > 0)
            {
                while (abs(phi_new) > pi222)
                {
                    phi_new -= 2 * pi222;
                }
            }
            else
            {
                while (abs(phi_new) > pi222)
                {
                    phi_new += 2 * pi222;
                }
            }

            if (std::abs(phi_new - phi) < tolerance) return phi_new;

            phi = phi_new;
        }
    }

public:
    Equation(double coming_angle, double coming_velocity, double paddle_velocity, double leaving_angle, double parameter_e) :coming_angle_(coming_angle* Deg2Rad), coming_velocity_(coming_velocity), paddle_velocity_(paddle_velocity), leaving_angle_(leaving_angle* Deg2Rad), parameter_e_(parameter_e)
    {
        paddle_angle_ = 0;
    }
    ~Equation() = default;

    double calculate_goA()
    {
        double initial_guess = (coming_angle_ + leaving_angle_) / 2;
        try {
            double paddle_angle_ = solve(coming_angle_, leaving_angle_, coming_velocity_, paddle_velocity_, parameter_e_, initial_guess);

            double residual = equation(paddle_angle_, coming_angle_, leaving_angle_, coming_velocity_, paddle_velocity_, parameter_e_);
            logger(Info, "Equation solving residual: ", residual);

            return paddle_angle_ * Rad2Deg;
        }
        catch (std::runtime_error)
        {
            logger(Error, "Solving error!");
            return initial_guess * Rad2Deg;
        }
    }
};

// calculate the angle to hit the ball
double calculate_hitting_angle(double ball_x, double ball_y, double posi_x, double posi_y, double coming_angle, double coming_velocity)
{
    double run_x = posi_x - ball_x;
    double run_y = posi_y - ball_y;
    double leaving_angle = atan2(run_y, run_x) * Rad2Deg;

    double paddle_velocity = config.speed * Deg2Rad * ((ball_y > 0 ? config.paddle_z_left : config.paddle_z_right) - 20);

    Equation equation(coming_angle, coming_velocity, paddle_velocity, leaving_angle, go_to_left ? config.parameter_e_left : config.parameter_e_right);
    double goA = equation.calculate_goA();

    logger(Info, "Coming angle: ", coming_angle, " degrees. Coming velocity: ", coming_velocity, " mm/s. Paddle velocity: ", paddle_velocity, " mm/s. Leaving angle: ", leaving_angle, " degrees. "
        , "Solution: paddle angle is ", goA, " degrees.");

    return goA - (ball_y > 0 ? config.jaka_angle_left : config.jaka_angle_right);
}


void batch_prediction()
{
    while (!is_hitting.get())
    {
        std::vector<Point> track = latest_ball_positions.snapshot();
        double x = 200;
        QuadraticPredictor qp(track);
        quadratic_predicted_y.set(qp.predict(x));
        LinearPredictor lp(track);
        linear_predicted_y.set(lp.predict(x));
    }

    logger(Error, "yeah4");
}

void half_quadratic_prediction_calculation()
{
    double x = 200;
    while (!is_hitting.get())
    {
        Sleep(20);
        while (half_table_points.snapshot().size() <= 5) {}
        QuadraticPredictor qp(half_table_points.snapshot());
        quadratic_predicted_y.set(qp.predict(x));
    }
}

void half_quadratic_prediction()
{
    half_table_points.clear();

    std::thread half_quadratic_prediction(half_quadratic_prediction_calculation);
    half_quadratic_prediction.detach();

    Point ball_position_copy;

    while (!is_hitting.get())
    {
        ball_position_copy = ball_position.get();
        Sleep(20);
        while (ball_position_copy.x >= 0.5 * TABLE_LENGTH) {}
        half_table_points.push(ball_position_copy);
    }
}

bool yeah;

void full_quadratic_prediction_calculation()
{
    double x = 200;
    while (yeah)
    {
        Sleep(20);
        QuadraticPredictor qp(full_table_points.snapshot());
        quadratic_predicted_y.set(qp.predict(x));
    }

    logger(Error, "yeah3");
}

void full_quadratic_prediction()
{
    if (full_table_points.empty()) full_table_points.clear();

    yeah = true;

    std::thread full_quadratic_prediction(full_quadratic_prediction_calculation);
    full_quadratic_prediction.detach();

    Point ball_position_copy;

    while (!is_hitting.get())
    {
        ball_position_copy = ball_position.get();
        Sleep(20);
        full_table_points.push(ball_position_copy);
    }

    yeah = false;

    logger(Error, "yeah2");
}

void record_trajectory_before()
{
    Point ball_position_copy;
    if (!is_hitting.get())
    {
        ball_position_copy = ball_position.get();
        trajectory_outfile << ball_position_copy.t << ',' << ball_position_copy.x << ',' << ball_position_copy.y << std::endl;
    }
}

void coming_calculation()
{
    while (!is_hitting.get())
    {
        std::vector<Point> track = latest_ball_positions.snapshot();
        coming_angle.set(get_ball_coming_direction(track));
        double vx = (track.front().x - track.back().x) / (track.back().t - track.front().t);
        double vy = (track.front().y - track.back().y) / (track.back().t - track.front().t);
        coming_velocity.set(sqrt(vx * vx + vy * vy));
        Sleep(20);
    }

    logger(Error, "yeah5");
}

void calculate_paddle_hitting()
{
    while (!is_hitting.get())
    {
        double y_quadratic_predicted = quadratic_predicted_y.get();
        double y_linear_predicted = linear_predicted_y.get();
        double y_predicted = y_linear_predicted * config.linear_weight_quadratic + y_quadratic_predicted * (1 - config.linear_weight_quadratic);
        int target_index = go_to_left ? 10 - config.goal_choice : config.goal_choice;
        hitting_angle.set(calculate_hitting_angle(200, y_predicted, goals[target_index].first, goals[target_index].second, coming_angle.get(), coming_velocity.get()));
    }

    logger(Error, "yeah6");
}

void trajectory_prediction()
{
    if (config.if_record_trajectory) {
        // 打开记录轨迹文件
        trajectory_outfile.open(string(config.repo_path) + "\\national\\dynamic\\data_trajectory.csv", std::ios::trunc);
        if (!trajectory_outfile.is_open()) {
            logger(Error, "Failed to open trajectory CSV file.");
        }
        trajectory_outfile << "t,x,y" << std::endl; // Write header

        std::thread record_trajectory_before_thread(record_trajectory_before);
        record_trajectory_before_thread.detach();
    }

    enum QuadraticMode { BATCH = 0, FULL = 1, HALF = 2 };
    QuadraticMode mode;

    std::thread batch_prediction_thread(batch_prediction);
    std::thread half_prediction_thread(half_quadratic_prediction);
    std::thread full_prediction_thread(full_quadratic_prediction);

    switch (config.quadratic_prediction_mode) {
    case 0: // batch quadratic prediction
        batch_prediction_thread.detach();
        mode = BATCH;
        break;

    case 1: // full-table quadratic prediction
        half_prediction_thread.detach();
        mode = FULL;
        break;

    case 2: // half-table quadratic prediction
    default:
        full_prediction_thread.detach();
        mode = HALF;
        break;
    }

    std::thread velocity_thread(coming_calculation);
    velocity_thread.detach();

    Sleep(20);

    std::thread hitting_thread(calculate_paddle_hitting);
    hitting_thread.detach();

    while (!is_hitting.get())
    {
        std::vector<Point> track = latest_ball_positions.snapshot();

        double y_quadratic_predicted = quadratic_predicted_y.get();
        double y_linear_predicted = linear_predicted_y.get();
        double y_predicted = y_linear_predicted * config.linear_weight_quadratic + y_quadratic_predicted * (1 - config.linear_weight_quadratic);

        go_to_left = (y_predicted > 0);

        JointValue temp;

        logger(Move, "Motor preparing.");
        if (is_hitting.get()) break;
        mkmotor.goToPos2(go_to_left ? -160 : -20, 1, 400, false);

        int target_index = go_to_left ? 10 - config.goal_choice : config.goal_choice;

        logger(Info, "Choose goal ", target_index, ".");


        CartesianPose pos;
        pos.tran.x = 200 - config.paddle_ball_center_distance * cos(hitting_angle.get() * Deg2Rad) + config.jaka_offset_x;
        pos.tran.y = y_predicted - config.paddle_ball_center_distance * sin(hitting_angle.get() * Deg2Rad) + config.jaka_offset_y;
        pos.tran.z = go_to_left ? config.paddle_z_left : config.paddle_z_right;
        pos.rpy.rx = go_to_left ? (-90 + hitting_angle.get()) * Deg2Rad : (90 - hitting_angle.get()) * Deg2Rad;
        pos.rpy.ry = go_to_left ? -90 * Deg2Rad : 90 * Deg2Rad;
        pos.rpy.rz = 0;
        try
        {
            demo.kine_inverse(&inijval, &pos, &jvalH);

            demo.joint_move(&jvalH, MoveMode::ABS, false, config.jaka_speed, 400 * Deg2Rad, 2, NULL);
            Sleep(50);
        }
        catch (std::runtime_error)
        {
            logger(Error, "Inverse error! Trying to move nearby.");

            if (y_predicted > 250 || (y_predicted > -250 && y_predicted < 0))
            {
                int index = 0;
                while (true)
                {
                    pos.tran.y -= 20;
                    try
                    {
                        demo.kine_inverse(&inijval, &pos, &jvalH);
                        demo.joint_move(&jvalH, MoveMode::ABS, false, config.jaka_speed, 400 * Deg2Rad, 2, NULL);
                        Sleep(200);
                        break;
                    }
                    catch (std::runtime_error)
                    {
                        index++;
                        if (index == 10)
                        {
                            logger(Error, "Inverse error! Cannot find a valid position.");
                            break;
                        }
                    }
                }
            }
            else
            {
                int index = 0;
                while (true)
                {
                    pos.tran.y += 20;
                    try
                    {
                        demo.kine_inverse(&inijval, &pos, &jvalH);
                        demo.joint_move(&jvalH, MoveMode::ABS, false, config.jaka_speed, 400 * Deg2Rad, 2, NULL);
                        Sleep(200);
                        break;
                    }
                    catch (std::runtime_error)
                    {
                        index++;
                        if (index == 10)
                        {
                            logger(Error, "Inverse error! Cannot find a valid position.");
                            break;
                        }
                    }
                }
            }
        }

        for (int aa = 0; aa < 6; aa++)
        {
            jval.jVal[aa] = jvalH.jVal[aa];
        }

        demo.servo_move_enable(FALSE);
    }
}

void calculate_hit(int round)
{
    logger(Info, "Round ", round, " hitting.");
    is_hitting.set(false);

    std::thread t(trajectory_prediction);
    t.detach(); // Start trajectory prediction in a separate thread

    // hitting!
    while (ball_position.get().x >= config.hitting_threshold && cmd.get() == 'k') {} // Wait for the ball to reach hitting position
    is_hitting.set(true);
    mkmotor.goToPos2(go_to_left ? -270 : 90, 1, config.speed, false);
    logger(Info, "Motor will hit when ball.x is ", ball_position.get().x);

    logger(Move, "Motor hitting.");

    std::thread t10(record_trajectory_after);
    t10.detach(); // Start recording trajectory in a separate thread
}

void get_correction_data()
{
    ifstream angle_file(string(config.repo_path) + "\\national\\dynamic\\data_GoalAngle.csv");

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

    logger(Info, "Main loop started.");

    std::thread t1(get_vision);
    t1.detach();
    logger(Info, "Vision system connected.");

    if (config.if_referee)
    {
        std::thread t2(get_referee);
        t2.detach();
        logger(Info, "Referee system connected.");
    }

    demo.login_in(config.jaka_ip);
    logger(Info, "Jaka mini connected.");

    demo.power_on();
    demo.enable_robot();
    logger(Info, "Jaka mini empowered.");

    mkmotor.ID = 1;
    mkmotor.reduceRatio = 10;
    mkmotor.setSerial(config.motor_com, 115200);
    mkmotor.start();
    logger(Info, "Motor connected.");

    inijval.jVal[0] = 0 * Deg2Rad;
    inijval.jVal[1] = 0 * Deg2Rad;
    inijval.jVal[2] = 90 * Deg2Rad;
    inijval.jVal[3] = 0 * Deg2Rad;
    inijval.jVal[4] = 0 * Deg2Rad;
    inijval.jVal[5] = 0 * Deg2Rad;

    logger(Move, "Jaka mini initializing.");
    demo.joint_move(&inijval, MoveMode::ABS, true, 5);

    logger(Move, "Motor initializing.");
    mkmotor.goToPos2(0, 1, 200, false);

    get_correction_data();
    int hitting_round = 1;

    if (!config.if_referee) cmd.set('k');

    while (cmd.get() != 'e') {
        if (cmd.get() != 'k') continue;

        while (!is_ball_moving_toward_robot()) {}
        calculate_hit(hitting_round);
        hitting_round++;

        Sleep(1000); // Wait for a while before the next hit
        logger(Move, "Jaka mini and motor returning.");
        mkmotor.goToPos2(0, 1, 200, false);
        demo.joint_move(&inijval, MoveMode::ABS, true, 5);

        if (config.if_referee) while (cmd.get() == 'k') {}
    }
}


void get_vision_vector()
{
    while (true)
    {
        if (ball_position.get().t != latest_ball_positions.snapshot().back().t)
        {
            latest_ball_positions.push(ball_position.get());
        }
    }
}

// Constantly get vision data in a separate thread
void get_vision()
{
    for (int i = 0; i < 15; i++)
    {
        latest_ball_positions.push(Point{ 1.0 + i,100.0 + i,0 });
    }

    RoboCupSSLClient client;
    client.open(true);
    SSL_WrapperPacket packet;

    std::thread vector_t(get_vision_vector);
    vector_t.detach();

    while (true)
    {
        if (client.receive(packet))
        {
            if (packet.has_detection())
            {
                // Check if the packet contains detection data
                SSL_DetectionFrame detection = packet.detection();
                balls_n.set(detection.balls_size());
                if (balls_n.get())
                {
                    SSL_DetectionBall ball = detection.balls(0);
                    Point pp;
                    pp.x = ball.x();
                    pp.y = ball.y();
                    pp.t = detection.t_capture();
                    ball_position.set(pp);
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
        cmd.set('H');
        break;
    case 1:
        cmd.set('S');
        break;
    case 2:
        cmd.set(' ');
        break;
    case 8:
        cmd.set('k');
        break;
    case 9:
        cmd.set('K');
        break;
    default:
        std::cout << "refereebox is fail !!!!! command : " << command << std::endl;
        cmd.set('H');
        break;
    }

    return cmd.get();
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