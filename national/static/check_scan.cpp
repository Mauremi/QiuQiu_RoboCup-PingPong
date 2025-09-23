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
    {4, {TABLE_LENGTH + 540,  250}},
    {5, {TABLE_LENGTH + 540, -250}},
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
    const double velocity_factor = 39.5; // velocity shift factor 35.5 for second targets

    // Paddle location
    const double paddle_z_left = 240, paddle_z_right = 240; // paddle height (when the ball is on the left or right side of the table)
    const double paddle_ball_center_distance = 10; // distance between paddle and ball center. Recommended value is 19 mm, which is the radius of the ball.
    const double jaka_offset_x = 0, jaka_offset_y = 0; // offset for jaka mini, used to adjust the paddle position

    // Settings (IFs)
    const bool if_record_trajectory = true; // if record trajectory
    const bool if_vision_correction = false;

    // Adjacent learning
    const bool if_adjacent_learning = true; // if use adjacent learning
    const int offset_class = 1; // 1: , 2: Exponential
    const double correction_coefficient_y = 0.8, correction_coefficient_x = 0.7;
    const int deviser_x = 100, deviser_y = 200; // Deviser for adjacent learning, used to calculate the distance
    const double fiction_factor = 0.86; // Fiction factor that affects the velocity when ball falling
} config;

// 在 enum Logger 后添加
enum ScanResult { Success, Stuck, OutOfReach };

// 扫描配置
struct ScanConfig {
    double y_start = -600, y_end = 600, y_step = 50;
    double x_start = -50, x_end = 300, x_step = 50;
    double speed = 5;
} scanConfig;


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
double hitting_angle;
char cmd = 'H'; // Command for the robot
std::mutex mtx;        // mutex for protecting shared resources

JAKAZuRobot demo;
MotorIK mkmotor;

void get_vision();      // vision thread function
void get_referee();

void clear_log_files() {
    auto clear_file = [](const std::string& path) {
        std::ofstream file(path, std::ios::trunc); // trunc 会清空文件
        if (file.is_open()) {
            file << ""; // 确保写入空内容
            file.close();
            logger(Info, "Cleared log file: ", path);
        }
        else {
            logger(Error, "Failed to clear log file: ", path);
        }
        };
    // 清除你关心的日志文件
    std::string repo_path(config.repo_path);
    clear_file(repo_path + "\\national\\static\\scan_log.csv");        // 如果你加了扫描功能
}

void scanWorkspace() {
    double last_x = 0, last_y = 0, last_z = 0;

    JointValue jval;
    jval.jVal[0] = 0 * Deg2Rad;
    jval.jVal[1] = 0 * Deg2Rad;
    jval.jVal[2] = 90 * Deg2Rad;
    jval.jVal[3] = 0 * Deg2Rad;
    jval.jVal[4] = 0 * Deg2Rad;
    jval.jVal[5] = 0 * Deg2Rad;

    logger(Hint, "Starting workspace scan (half-axis priority logic).");

    std::ofstream log_file(string(config.repo_path) + "\\national\\static\\scan_log.csv");
    if (!log_file.is_open()) {
        logger(Error, "Failed to open scan log file!");
        return;
    }
    log_file << "col,row,target_x,target_y,target_z,actual_x,actual_y,actual_z,status\n";

    int col = 0;

    for (double x = scanConfig.x_start; x <= scanConfig.x_end; x += scanConfig.x_step) {
        int sign = 1;

        double y_max = scanConfig.y_end;

        // ========= 1. 从 |y| 最大值向 0 扫描 =========
        bool found_outer_success = false;
        int left_outer_row = -1;

        for (double y = y_max; sign * y > 0; y -= sign * scanConfig.y_step) {
            if (found_outer_success) break;

            CartesianPose pos;
            pos.tran.x = x;
            pos.tran.y = y;
            pos.tran.z = (y > 0) ? config.paddle_z_left : config.paddle_z_right;
            pos.rpy.rx = ((y > 0) ? -90 : 90) * Deg2Rad;
            pos.rpy.ry = ((y > 0) ? -90 : 90) * Deg2Rad;
            pos.rpy.rz = 0;



            JointValue jvalHome;
            try {
                demo.kine_inverse(&jval, &pos, &jvalHome);
            }
            catch (const std::runtime_error& e) {
                logger(Error, "IK failed at (", x, ",", y, ")");
                log_file << col << "," << (int)((y - scanConfig.y_start) / scanConfig.y_step)
                    << "," << x << "," << y << "," << pos.tran.z
                    << ",,,,IK_Fail\n";
                col++;
                continue;
            }

            demo.joint_move(&jvalHome, MoveMode::ABS, false, scanConfig.speed);
            mkmotor.goToPos2((sign > 0) ? 180 : 0, 1, false);

            char response;
            logger(Input, "Did robot reach (", x, ", ", y, ")? (y/n): ");
            std::cin >> response;

            if (response == 'y') {
                found_outer_success = true;
                left_outer_row = (int)((y - scanConfig.y_start) / scanConfig.y_step);
                last_x = x; last_y = y; last_z = pos.tran.z;

                log_file << col << "," << left_outer_row << "," << x << "," << y << "," << pos.tran.z
                    << "," << x << "," << y << "," << pos.tran.z << ",Success\n";
            }
            else {
                log_file << col << "," << (int)((y - scanConfig.y_start) / scanConfig.y_step)
                    << "," << x << "," << y << "," << pos.tran.z
                    << "," << last_x << "," << last_y << "," << last_z << ",Stuck\n";
            }
            col++;
        }

        sign = -1;

        y_max = scanConfig.y_start;

        // ========= 2. 从 y=0 向外扫描 =========
        bool found_inner_success = false;
        int right_inner_row = -1;

        for (double y = 0; sign * y < sign * y_max; y += sign * scanConfig.y_step) {
            if (found_inner_success) break;

            CartesianPose pos;
            pos.tran.x = x;
            pos.tran.y = y;
            pos.tran.z = (y > 0) ? config.paddle_z_left : config.paddle_z_right;
            pos.rpy.rx = ((y > 0) ? -90 : 90) * Deg2Rad;
            pos.rpy.ry = ((y > 0) ? -90 : 90) * Deg2Rad;
            pos.rpy.rz = 0;

            JointValue jvalHome;
            try {
                demo.kine_inverse(&jval, &pos, &jvalHome);
            }
            catch (const std::runtime_error& e) {
                logger(Error, "IK failed at (", x, ",", y, ")");
                log_file << col << "," << (int)((y - scanConfig.y_start) / scanConfig.y_step)
                    << "," << x << "," << y << "," << pos.tran.z
                    << ",,,,IK_Fail\n";
                col++;
                continue;
            }

            demo.joint_move(&jvalHome, MoveMode::ABS, false, scanConfig.speed);

            char response;
            logger(Input, "Did robot reach (", x, ", ", y, ")? (y/n): ");
            std::cin >> response;

            if (response == 'y') {
                found_inner_success = true;
                right_inner_row = (int)((y - scanConfig.y_start) / scanConfig.y_step);
                last_x = x; last_y = y; last_z = pos.tran.z;

                log_file << col << "," << right_inner_row << "," << x << "," << y << "," << pos.tran.z
                    << "," << x << "," << y << "," << pos.tran.z << ",Success\n";
            }
            else {
                log_file << col << "," << (int)((y - scanConfig.y_start) / scanConfig.y_step)
                    << "," << x << "," << y << "," << pos.tran.z
                    << "," << last_x << "," << last_y << "," << last_z << ",Stuck\n";
            }
            col++;
        }

        sign = +1;

        y_max = scanConfig.y_end;

        // ========= 2. 从 y=0 向外扫描 =========
        found_inner_success = false;
        int left_inner_row = -1;

        for (double y = 0; sign * y < sign * y_max; y += sign * scanConfig.y_step) {
            if (found_inner_success) break;

            CartesianPose pos;
            pos.tran.x = x;
            pos.tran.y = y;
            pos.tran.z = (y > 0) ? config.paddle_z_left : config.paddle_z_right;
            pos.rpy.rx = ((y > 0) ? -90 : 90) * Deg2Rad;
            pos.rpy.ry = ((y > 0) ? -90 : 90) * Deg2Rad;
            pos.rpy.rz = 0;

            JointValue jvalHome;
            try {
                demo.kine_inverse(&jval, &pos, &jvalHome);
            }
            catch (const std::runtime_error& e) {
                logger(Error, "IK failed at (", x, ",", y, ")");
                log_file << col << "," << (int)((y - scanConfig.y_start) / scanConfig.y_step)
                    << "," << x << "," << y << "," << pos.tran.z
                    << ",,,,IK_Fail\n";
                col++;
                continue;
            }

            demo.joint_move(&jvalHome, MoveMode::ABS, false, scanConfig.speed);
            mkmotor.goToPos2((sign > 0) ? 180 : 0, 1, false);

            char response;
            logger(Input, "Did robot reach (", x, ", ", y, ")? (y/n): ");
            std::cin >> response;

            if (response == 'y') {
                found_inner_success = true;
                left_inner_row = (int)((y - scanConfig.y_start) / scanConfig.y_step);
                last_x = x; last_y = y; last_z = pos.tran.z;

                log_file << col << "," << left_inner_row << "," << x << "," << y << "," << pos.tran.z
                    << "," << x << "," << y << "," << pos.tran.z << ",Success\n";
            }
            else {
                log_file << col << "," << (int)((y - scanConfig.y_start) / scanConfig.y_step)
                    << "," << x << "," << y << "," << pos.tran.z
                    << "," << last_x << "," << last_y << "," << last_z << ",Stuck\n";
            }
            col++;
        }


        sign = -1;

        y_max = scanConfig.y_start;

        // ========= 1. 从 |y| 最大值向 0 扫描 =========
        found_outer_success = false;
        int right_outer_row = -1;

        for (double y = y_max; sign * y > 0; y -= sign * scanConfig.y_step) {
            if (found_outer_success) break;

            CartesianPose pos;
            pos.tran.x = x;
            pos.tran.y = y;
            pos.tran.z = (y > 0) ? config.paddle_z_left : config.paddle_z_right;
            pos.rpy.rx = ((y > 0) ? -90 : 90) * Deg2Rad;
            pos.rpy.ry = ((y > 0) ? -90 : 90) * Deg2Rad;
            pos.rpy.rz = 0;



            JointValue jvalHome;
            try {
                demo.kine_inverse(&jval, &pos, &jvalHome);
            }
            catch (const std::runtime_error& e) {
                logger(Error, "IK failed at (", x, ",", y, ")");
                log_file << col << "," << (int)((y - scanConfig.y_start) / scanConfig.y_step)
                    << "," << x << "," << y << "," << pos.tran.z
                    << ",,,,IK_Fail\n";
                col++;
                continue;
            }

            demo.joint_move(&jvalHome, MoveMode::ABS, false, scanConfig.speed);
            mkmotor.goToPos2((sign > 0) ? 180 : 0, 1, false);

            char response;
            logger(Input, "Did robot reach (", x, ", ", y, ")? (y/n): ");
            std::cin >> response;

            if (response == 'y') {
                found_outer_success = true;
                left_outer_row = (int)((y - scanConfig.y_start) / scanConfig.y_step);
                last_x = x; last_y = y; last_z = pos.tran.z;

                log_file << col << "," << left_outer_row << "," << x << "," << y << "," << pos.tran.z
                    << "," << x << "," << y << "," << pos.tran.z << ",Success\n";
            }
            else {
                log_file << col << "," << (int)((y - scanConfig.y_start) / scanConfig.y_step)
                    << "," << x << "," << y << "," << pos.tran.z
                    << "," << last_x << "," << last_y << "," << last_z << ",Stuck\n";
            }
            col++;
        }


        // ========= 3. 自动填充中间区域 =========
        if (left_outer_row >= 0 && left_inner_row >= 0) {
            for (int row = std::min<int>(left_outer_row, left_inner_row) + 1; row < std::max<int>(left_outer_row, left_inner_row); row++) {
                double y_mid = scanConfig.y_start + row * scanConfig.y_step;
                double z_mid = (y_mid > 0) ? config.paddle_z_left : config.paddle_z_right;
                log_file << col << "," << row << "," << x << "," << y_mid << "," << z_mid
                    << "," << x << "," << y_mid << "," << z_mid << ",Success\n";
                col++;
            }
        }

        if (right_outer_row >= 0 && right_inner_row >= 0) {
            for (int row = std::min<int>(right_outer_row, right_inner_row) + 1; row < std::max<int>(right_outer_row, right_inner_row); row++) {
                double y_mid = scanConfig.y_start + row * scanConfig.y_step;
                double z_mid = (y_mid > 0) ? config.paddle_z_left : config.paddle_z_right;
                log_file << col << "," << row << "," << x << "," << y_mid << "," << z_mid
                    << "," << x << "," << y_mid << "," << z_mid << ",Success\n";
                col++;
            }
        }
    }

    log_file.close();
    logger(Info, "Scan completed with half-axis priority logic. Log saved to scan_log.csv");
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

    // Separate thread for vision system
    std::thread t1(get_vision);
    t1.detach();
    logger(Info, "Vision system connected.");

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

    scanWorkspace();


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
        ifstream correction_file(string(config.repo_path) + "\\national\\public\\data_VisionCorrection.csv");
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