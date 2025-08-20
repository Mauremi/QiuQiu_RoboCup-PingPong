#include <stdio.h>
#include <cmath>

// #include "timer.h"
// #pragma warning(disable:4996)
#include "net/robocup_ssl_client.h"
// #include "timer.h"
// #pragma warning(disable:4996)
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
#include <iostream>
#include <thread>
#include <mutex>
#define pi222 3.1415926
#define M_PI pi222
#define Deg2Rad (pi222 / 180)
#define Rad2Deg (180 / pi222)
#include <iostream>
#include <cmath>
#include <vector>

#include <iostream>
#include <cmath>
#include <vector>

#include <fstream>
#include <iostream>

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <complex>

struct Point
{
    double x;
    double y;
    double t; // time
};
Point ballPos;
std::mutex mtx;
std::ofstream outfile;
JAKAZuRobot demo;
MotorIK mkmotor;

char cmd = 'H';
double goA = -10; // 击球角度
JointValue jval, jvalH, inijval;
void getVision();
void getReferee();
double goAvalue(double ball_x, double ball_y, double posi_x, double posi_y, double angle);
int choice(std::vector<Point> ball, double pre_x, double pre_y);
double get_hit_speed_direction(std::vector<Point> balls);
//double speedX(double predict_x, double predict_y, int tong, std::vector<Point> ball,angle);
double speedX(double ball_x, double ball_y, double posi_x, double posi_y, double angle);
double getdistance(double ball_x, double ball_y, double posi_x, double posi_y);
std::map<int, int> hit_score; // 第几轮击球和每轮的得分
std::map<int, std::pair<double, double>> tong_position = {
    {1, {877.56, 0}},
    {2, {640, 250}},
    {3, {640, -250}},
    {4, {278.88, 563.88}},
    {5, {278.88, -563.88}},
    {6, {278.88, 0}}}; // 目标桶的序号和他的坐标

class LinearPredictor
{
public:
    LinearPredictor(const std::vector<Point> &points)
    {
        double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
        int n = points.size();

        for (const auto &point : points)
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

    double predict(double x)
    {
        // Predict y value for given x
        return slope * x + intercept;
    }

private:
    double slope;
    double intercept;
};

class QuadraticPredictor
{
public:
    QuadraticPredictor(const std::vector<Point> &points)
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

private:
    double a;       // Coefficient for x^2
    double b_coeff; // Coefficient for x
    double c;       // Constant term
};

class JointSpeedPlanner
{
private:
    double max_acceleration;   // 最大加速度
    double max_velocity;       // 最大速度
    double max_velocity_angle; // 达到最大速度时的角度

public:
    JointSpeedPlanner(double max_acc, double max_vel, double velocity_angle)
        : max_acceleration(max_acc), max_velocity(max_vel), max_velocity_angle(velocity_angle) {}

    // 计算时间段内的加速度
    double computeAcceleration(double t)
    {
        if (t < max_velocity / max_acceleration)
        {
            return max_acceleration;
        }
        else
        {
            return -max_acceleration;
        }
    }

    // 计算在特定时间内的角度
    double computeAngle(double t, double initial_angle)
    {
        double acceleration = computeAcceleration(t);
        if (acceleration == max_acceleration)
        {
            return initial_angle + 0.5 * max_acceleration * t * t;
        }
        else if (acceleration == -max_acceleration)
        {
            double t1 = max_velocity / max_acceleration;
            double t2 = 2 * max_velocity_angle / max_velocity - t1;
            double t3 = 2 * t1 - t;
            return initial_angle + max_velocity * t1 - 0.5 * max_acceleration * t3 * t3;
        }
        else
        {
            double t1 = max_velocity / max_acceleration;
            return initial_angle + max_velocity * t1 - 0.5 * max_acceleration * (t - t1) * (t - t1);
        }
    }

    // 生成角度序列
    std::vector<double> generateAngleSequence(double time_step, double initial_angle)
    {
        initial_angle = max_velocity_angle - 0.5 * max_velocity * max_velocity / max_acceleration;
        std::vector<double> angles;
        double current_time = 0.0;
        while (current_time <= 2 * max_velocity / max_acceleration)
        {
            angles.push_back(computeAngle(current_time, initial_angle));
            current_time += time_step;
        }
        return angles;
    }
    // 重载generateAngleSequence函数，返回角度和速度序列
    std::pair<std::vector<double>, std::vector<double>> generateAngleSequence2(double time_step, double initial_angle)
    {
        std::vector<double> angles;
        std::vector<double> velocities;
        initial_angle = max_velocity_angle - 0.5 * max_velocity * max_velocity / max_acceleration;
        double current_time = 0.0;
        while (current_time <= 2 * max_velocity / max_acceleration)
        {
            double angle = computeAngle(current_time, initial_angle);
            angles.push_back(angle);
            velocities.push_back(computeVelocity(current_time));
            current_time += time_step;
        }
        return std::make_pair(angles, velocities);
    }

    // 计算在特定时间内的速度
    double computeVelocity(double t)
    {
        if (t < max_velocity / max_acceleration)
        {
            return max_acceleration * t;
        }
        else
        {
            double t1 = max_velocity / max_acceleration;
            double t2 = 2 * max_velocity_angle / max_velocity - t1;
            return max_acceleration * (2 * t1 - t);
        }
    }
};

uint64_t getMs()
{
    FILETIME ft;
    GetSystemTimeAsFileTime(&ft); // 获取当前系统时间

    // 将FILETIME转换为64位整数
    ULARGE_INTEGER ui;
    ui.LowPart = ft.dwLowDateTime;
    ui.HighPart = ft.dwHighDateTime;

    // 将FILETIME转换为微秒
    uint64_t microseconds = (ui.QuadPart - 116444736000000000ULL) / 10;
    return microseconds;
}

//int choice(std::map<int, int> hit_score)
//{
//    return 3;
//}

void inverse_error_process(int lun)
{
    cout << "inverse error" << endl;
}

//以下为调参方便用宏定义
#define RIGHT_LINE 0.7//线性部分的权重
#define X_TRANSFORM 20//机械臂末端x需要在球后多少
#define Z_HEIGHT_LEFT 235//左侧z值
#define Z_HEIGHT_RIGHT 230//右侧z值

string fname = "robotandmoter.txt";



void calculate_hit()
{
    errno_t ret = ERR_SUCC;
    JointSpeedPlanner plannerLK(9000, 850, 180);
    std::vector<double> angle_sequenceLK;
    std::vector<double> velocity_sequenceLK;
    std::tie(angle_sequenceLK, velocity_sequenceLK) = plannerLK.generateAngleSequence2(0.008, 0); // 时间间隔为8ms

    std::reverse(angle_sequenceLK.begin(), angle_sequenceLK.end());
    std::reverse(velocity_sequenceLK.begin(), velocity_sequenceLK.end());

    double temp1 = angle_sequenceLK[0];
    mkmotor.goToPos2(temp1, 1, 100, false);
    Sleep(1000);
    std::cout << "ok, wait ball" << std::endl;

    double lastT = 0;
    double lastY = 0;
    std::vector<Point> ballP2;
    int count = 0;
    int maxN = 10;
    uint64_t time_Start, time_end;

    // 初步调整
    cout<<"初步调整"<<endl;
    while (count < maxN)
    {

        mtx.lock();
        if (ballPos.t != lastT && ballPos.y >= -750 && ballPos.y <= 750 && ballPos.x < 2150 && ballPos.x > 1600)
        {
            ballP2.push_back(ballPos);
            // time_Start = time(0();
            if (count == 0)
                time_Start = getMs();
            lastT = ballPos.t;
            // lastY = ballP
            count++;
            if (count == maxN)
                time_end = getMs();

            cout << ballPos.x << " " << ballPos.y << endl;
        }
        mtx.unlock();
        Sleep(5);
    }

    double hit_angle = get_hit_speed_direction(ballP2); // 乒乓球到达指定位置时的角度，即轨迹与x轴的夹角（锐角）
    LinearPredictor lp(ballP2);
    QuadraticPredictor qp(ballP2);

    double x = 100;
    double y = lp.predict(x);
    double yy = qp.predict(x);
    double posi = y*RIGHT_LINE+yy*(1-RIGHT_LINE);


    int tong_posi = 6;
    cout << "选择击球桶的编号为 " << tong_posi << endl;

    //一轮粗调确定左右位置
    //如果在中间，直接定点平推？
    JointValue temp;
    if (posi > 0) {
        //temp.jVal[0] = 61.638 * Deg2Rad;
        //temp.jVal[1] = 10.144 * Deg2Rad;
        //temp.jVal[2] = 70.353 * Deg2Rad;
        //temp.jVal[3] = -100.183 * Deg2Rad;
        //temp.jVal[4] = -19.455 * Deg2Rad;
        //temp.jVal[5] = -293.661 * Deg2Rad;
        //ret = demo.joint_move(&temp, MoveMode::ABS, false, 5, 400 * Deg2Rad, 2, NULL);
        mkmotor.goToPos2(250, 0, 600, false);
        goA = goAvalue(x, posi, tong_position[tong_posi].first, tong_position[tong_posi].second, hit_angle);
        cout << tong_position[tong_posi].first << ' ' << tong_position[tong_posi].second << endl;

        CartesianPose pos;
        pos.tran.x = x- X_TRANSFORM;
        pos.tran.y = posi;
        pos.tran.z = Z_HEIGHT_LEFT;
        pos.rpy.rx = (-90 - goA) * Deg2Rad;
        pos.rpy.ry = -90 * Deg2Rad;
        pos.rpy.rz = 0;
        ret = demo.kine_inverse(&inijval, &pos, &jvalH);
        for (int aa = 0; aa < 6; aa++)
        {
            jval.jVal[aa] = jvalH.jVal[aa];
        }
        if (ret == 0)
        {
            // for (int j = 0; j < 6; j++)
            //     std::cout << jval.jVal[j] * Rad2Deg << std::endl;
            demo.servo_move_enable(FALSE);
            ret = demo.joint_move(&jvalH, MoveMode::ABS, false, 5, 400 * Deg2Rad, 2, NULL);
        }
        else
        {
            cout << "inverse error" << endl;
            inverse_error_process(1);
        }

        // 第二轮调整
        while (ballPos.x >= 1550)
        {
        };
        cout << "第二轮调整" << endl;
        std::vector<Point> ballP2new;
        count = 0;
        maxN = 20;


        while (count < maxN)
        {

            mtx.lock();
            if (ballPos.t != lastT && ballPos.y >= -750 && ballPos.y <= 750 && ballPos.x < 1550 && ballPos.x > 1000)
            {
                ballP2new.push_back(ballPos);
                // time_Start = time(0();
                if (count == 0)
                    time_Start = getMs();
                lastT = ballPos.t;
                // lastY = ballP
                count++;
                if (count == maxN)
                    time_end = getMs();

                cout << ballPos.x << " " << ballPos.y << endl;
            }
            mtx.unlock();
            Sleep(5);
        }

        hit_angle = get_hit_speed_direction(ballP2new); // 乒乓球到达指定位置时的角度，即轨迹与x轴的夹角（锐角）

        uint64_t go_timenew = time_Start + (time_end - time_Start) / (ballP2new[1].x - ballP2new[maxN - 1].x) * (ballP2new[1].x - 100) * 0.5;
        LinearPredictor lp_new(ballP2new);
        QuadraticPredictor qp_new(ballP2new);

        double y_new = lp_new.predict(x);
        double yy_new = qp_new.predict(x);
        double posi_new = y_new*RIGHT_LINE + yy_new*(1-RIGHT_LINE) ;

        goA = goAvalue(x, posi_new, tong_position[tong_posi].first, tong_position[tong_posi].second, hit_angle);

        cout << goA << endl;

        pos.tran.x = x- X_TRANSFORM;
        pos.tran.y = posi_new;
        pos.tran.z = Z_HEIGHT_LEFT;
        pos.rpy.rx = (-90 - goA) * Deg2Rad;
        pos.rpy.ry = -90 * Deg2Rad;
        pos.rpy.rz = 0;
        ret = demo.kine_inverse(&jvalH, &pos, &jvalH);
        for (int aa = 0; aa < 6; aa++)
        {
            jval.jVal[aa] = jvalH.jVal[aa];
        }
        if (ret == 0)
        {
            // for (int j = 0; j < 6; j++)
            //     std::cout << jval.jVal[j] * Rad2Deg << std::endl;
            demo.servo_move_enable(FALSE);
            ret = demo.joint_move(&jvalH, MoveMode::ABS, false, 5, 400 * Deg2Rad, 2, NULL);
        }
        else
        {
            cout << "inverse error" << endl;
            // 如果求不出逆解，进行故障处理
            inverse_error_process(2);
        }

        // 第三轮调整，希望能拿到击球点前30cm到50cm的球位置数据，来进一步精确击球点和击球角度、速度
        while (ballPos.x >= 1000)
        {
        };
        cout << "第三轮调整" << endl;

        std::vector<Point> ballP2new2;
        count = 0;
        maxN = 10;

        while (count < maxN)
        {
            mtx.lock();
            if (ballPos.t != lastT && ballPos.y >= -750 && ballPos.y <= 750 && ballPos.x < 700 && ballPos.x > 400)
            {
                ballP2new2.push_back(ballPos);
                // time_Start = time(0();
                if (count == 0)
                    time_Start = getMs();
                lastT = ballPos.t;
                // lastY = ballP
                count++;
                if (count == maxN)
                    time_end = getMs();

                cout << ballPos.x << " " << ballPos.y << endl;
            }
            mtx.unlock();
            Sleep(5);
        }

        uint64_t go_timenew2 = time_Start + (time_end - time_Start) / (ballP2new2[1].x - ballP2new2[maxN - 1].x) * (ballP2new2[1].x - 100) * 1.02;
        hit_angle = get_hit_speed_direction(ballP2new2); // 乒乓球到达指定位置时的角度，即轨迹与x轴的夹角（锐角）

        LinearPredictor lp_new2(ballP2new2);
        QuadraticPredictor qp_new2(ballP2new2);

        double y_new2 = lp_new2.predict(x);
        double yy_new2 = qp_new2.predict(x);
        double posi_new2 = y_new2*RIGHT_LINE + yy_new2*(1-RIGHT_LINE);

        goA = goAvalue(x, posi_new2, tong_position[tong_posi].first, tong_position[tong_posi].second, hit_angle);

        cout << goA << endl;

        pos.tran.x = x - X_TRANSFORM;
        pos.tran.y = posi_new2;
        pos.tran.z = Z_HEIGHT_LEFT;
        pos.rpy.rx = (-90 - goA) * Deg2Rad;
        pos.rpy.ry = -90 * Deg2Rad;
        pos.rpy.rz = 0;
        ret = demo.kine_inverse(&jvalH, &pos, &jvalH);
        for (int aa = 0; aa < 6; aa++)
        {
            jval.jVal[aa] = jvalH.jVal[aa];
        }
        if (ret == 0)
        {
            // for (int j = 0; j < 6; j++)
            //     std::cout << jval.jVal[j] * Rad2Deg << std::endl;
            demo.servo_move_enable(FALSE);
            ret = demo.joint_move(&jvalH, MoveMode::ABS, false, 5, 400 * Deg2Rad, 2, NULL);
        }
        else
        {
            cout << "inverse error" << endl;
            // 如果求不出逆解，进行故障处理
            inverse_error_process(3);
        }



        // 时间计算与击球函数
        //while (getMs() < go_timenew2 - angle_sequenceLK.size() / 2 * 8 * 1000)
        //{

        //    Sleep(1);
        //}
        // shoot
        //for (int j = 0; j < angle_sequenceLK.size(); j++)
        //{
        //    double angles = angle_sequenceLK[angle_sequenceLK.size() - 1];
        //    double speed = velocity_sequenceLK[j];
        //    // std::cout << "LK : " << i * 0.008 << " seconds, Angle: " << angle_sequenceLK[j] << " radians" << std::endl;
        //    if (speed < 1)
        //        speed = 200;
        //    // angles[i] = angles[85];

        //    mkmotor.goToPos2(angles, 1, speed, false);

        //    Sleep(7);
        //}
        //cout << "当前x坐标" << ballPos.x << endl;
        while (ballPos.x >= 250) {

        };

        //ret = demo.servo_move_use_joint_LPF(2); // 使用LPF滤波器
        //demo.servo_move_enable(TRUE);
        double speed = speedX(85, posi_new2, tong_position[tong_posi].first, tong_position[tong_posi].second, hit_angle);
        cout << "speed: " <<speed<< endl;
       // cout << "当前x坐标" << ballPos.x << endl;
        mkmotor.goToPos2(180, 1, speed, false);

        //ofstream outFile(fname, ios::app);
        //CartesianPose* temp;
        //ret = demo.get_tcp_position(temp);
        //outFile << "电机计算速度为：" << to_string(speed) << endl << "机器人位姿：" << endl << temp->tran.x << endl << temp->tran.y << endl << temp->tran.z << temp->rpy.rx << endl << temp->rpy.ry << endl << temp->rpy.rz << endl;
        //outFile.close();
        //Sleep(1000);
        //ret = demo.joint_move(&temp, MoveMode::ABS, false, 5, 400 * Deg2Rad, 2, NULL);
    //}
    //while (ballPos.x >= 200) {};
    //    mkmotor.goToPos2(180, 1, 600, false);
    }

    else 
    {
        //temp.jVal[0] = -90 * Deg2Rad;
        //temp.jVal[1] = 0 * Deg2Rad;
        //temp.jVal[2] = 90 * Deg2Rad;
        //temp.jVal[3] = 0 * Deg2Rad;
        //temp.jVal[4] = 0 * Deg2Rad;
        //temp.jVal[5] = 0 * Deg2Rad;
        //ret = demo.joint_move(&temp, MoveMode::ABS, false, 5, 400 * Deg2Rad, 2, NULL);
        mkmotor.goToPos2(100, 1, 600, false);
        goA = goAvalue(x, posi, tong_position[tong_posi].first, tong_position[tong_posi].second, hit_angle);
        CartesianPose pos;
        pos.tran.x = x- X_TRANSFORM;
        pos.tran.y = posi;
        pos.tran.z = Z_HEIGHT_RIGHT;
        pos.rpy.rx = (90 + goA) * Deg2Rad;
        pos.rpy.ry = -90 * Deg2Rad;
        pos.rpy.rz = 0;
        ret = demo.kine_inverse(&inijval, &pos, &jvalH);
        for (int aa = 0; aa < 6; aa++)
        {
            jval.jVal[aa] = jvalH.jVal[aa];
        }
        if (ret == 0)
        {
            // for (int j = 0; j < 6; j++)
            //     std::cout << jval.jVal[j] * Rad2Deg << std::endl;
            demo.servo_move_enable(FALSE);
            ret = demo.joint_move(&jvalH, MoveMode::ABS, false, 5, 400 * Deg2Rad, 2, NULL);
        }
        else
        {
            cout << "inverse error" << endl;
            inverse_error_process(1);
        }

        // 第二轮调整
        while (ballPos.x >= 1550)
        {
        };
        cout << "第二轮调整" << endl;
        std::vector<Point> ballP2new;
        count = 0;
        maxN = 20;


        while (count < maxN)
        {

            mtx.lock();
            if (ballPos.t != lastT && ballPos.y >= -750 && ballPos.y <= 750 && ballPos.x < 1550 && ballPos.x > 1000)
            {
                ballP2new.push_back(ballPos);
                // time_Start = time(0();
                if (count == 0)
                    time_Start = getMs();
                lastT = ballPos.t;
                // lastY = ballP
                count++;
                if (count == maxN)
                    time_end = getMs();

                cout << ballPos.x << " " << ballPos.y << endl;
            }
            mtx.unlock();
            Sleep(5);
        }

        hit_angle = get_hit_speed_direction(ballP2new); // 乒乓球到达指定位置时的角度，即轨迹与x轴的夹角（锐角）

       
        LinearPredictor lp_new(ballP2new);
        QuadraticPredictor qp_new(ballP2new);

        double y_new = lp_new.predict(x);
        double yy_new = qp_new.predict(x);
        double posi_new = y_new * RIGHT_LINE + yy_new * (1 - RIGHT_LINE);

        goA = goAvalue(x, posi_new, tong_position[tong_posi].first, tong_position[tong_posi].second, hit_angle);

        cout << goA << endl;

        pos.tran.x = x- X_TRANSFORM;
        pos.tran.y = posi_new;
        pos.tran.z = Z_HEIGHT_RIGHT;
        pos.rpy.rx = (90 + goA) * Deg2Rad;
        pos.rpy.ry = -90 * Deg2Rad;
        pos.rpy.rz = 0;
        ret = demo.kine_inverse(&jvalH, &pos, &jvalH);
        for (int aa = 0; aa < 6; aa++)
        {
            jval.jVal[aa] = jvalH.jVal[aa];
        }
        if (ret == 0)
        {
            // for (int j = 0; j < 6; j++)
            //     std::cout << jval.jVal[j] * Rad2Deg << std::endl;
            demo.servo_move_enable(FALSE);
            ret = demo.joint_move(&jvalH, MoveMode::ABS, false, 5, 400 * Deg2Rad, 2, NULL);
        }
        else
        {
            cout << "inverse error" << endl;
            // 如果求不出逆解，进行故障处理
            inverse_error_process(2);
        }

        // 第三轮调整，希望能拿到击球点前30cm到50cm的球位置数据，来进一步精确击球点和击球角度、速度
        while (ballPos.x >= 1000)
        {
        };
        cout << "第三轮调整" << endl;

        std::vector<Point> ballP2new2;
        count = 0;
        maxN = 10;

        while (count < maxN)
        {
            mtx.lock();
            if (ballPos.t != lastT && ballPos.y >= -750 && ballPos.y <= 750 && ballPos.x < 700 && ballPos.x > 400)
            {
                ballP2new2.push_back(ballPos);
                // time_Start = time(0();
                if (count == 0)
                    time_Start = getMs();
                lastT = ballPos.t;
                // lastY = ballP
                count++;
                if (count == maxN)
                    time_end = getMs();

                cout << ballPos.x << " " << ballPos.y << endl;
            }
            mtx.unlock();
            Sleep(5);
        }

        uint64_t go_timenew2 = time_Start + (time_end - time_Start) / (ballP2new2[1].x - ballP2new2[maxN - 1].x) * (ballP2new2[1].x - 100) * 1;
        hit_angle = get_hit_speed_direction(ballP2new2); // 乒乓球到达指定位置时的角度，即轨迹与x轴的夹角（锐角）

        LinearPredictor lp_new2(ballP2new2);
        QuadraticPredictor qp_new2(ballP2new2);

        double y_new2 = lp_new2.predict(x);
        double yy_new2 = qp_new2.predict(x);
        double posi_new2 = y_new2 * RIGHT_LINE + yy_new2 * (1 - RIGHT_LINE);

        goA = goAvalue(x, posi_new2, tong_position[tong_posi].first, tong_position[tong_posi].second, hit_angle);

        cout << goA << endl;

        pos.tran.x = x - X_TRANSFORM;
        pos.tran.y = posi_new2;
        pos.tran.z = Z_HEIGHT_RIGHT;
        pos.rpy.rx = (90 + goA) * Deg2Rad;
        pos.rpy.ry = -90 * Deg2Rad;
        pos.rpy.rz = 0;
        ret = demo.kine_inverse(&jvalH, &pos, &jvalH);
        for (int aa = 0; aa < 6; aa++)
        {
            jval.jVal[aa] = jvalH.jVal[aa];
        }
        if (ret == 0)
        {
            // for (int j = 0; j < 6; j++)
            //     std::cout << jval.jVal[j] * Rad2Deg << std::endl;
            demo.servo_move_enable(FALSE);
            ret = demo.joint_move(&jvalH, MoveMode::ABS, false, 5, 400 * Deg2Rad, 2, NULL);
        }
        else
        {
            cout << "inverse error" << endl;
            // 如果求不出逆解，进行故障处理
            inverse_error_process(3);
        }



        // 时间计算与击球函数
        //while (getMs() < go_timenew2 - angle_sequenceLK.size() / 2 * 8 * 1000)
        //{

        //    Sleep(1);
        //}
        // shoot
        //for (int j = 0; j < angle_sequenceLK.size(); j++)
        //{
        //    double angles = angle_sequenceLK[angle_sequenceLK.size() - 1];
        //    double speed = velocity_sequenceLK[j];
        //    // std::cout << "LK : " << i * 0.008 << " seconds, Angle: " << angle_sequenceLK[j] << " radians" << std::endl;
        //    if (speed < 1)
        //        speed = 200;
        //    // angles[i] = angles[85];

        //    mkmotor.goToPos2(angles, 1, speed, false);

        //    Sleep(7);
        //}
        while (ballPos.x >= 250) {
   
        };
       
        //ret = demo.servo_move_use_joint_LPF(2); // 使用LPF滤波器
        //demo.servo_move_enable(TRUE);
        double speed = speedX(85, posi_new2, tong_position[tong_posi].first, tong_position[tong_posi].second, hit_angle);
        mkmotor.goToPos2(200, 1, speed, false);

        /*ofstream outFile(fname, ios::app);
        CartesianPose* temp;
        errno_t ret1 = ERR_SUCC;
        ret1 = demo.get_tcp_position(temp);
        outFile << "电机计算速度为：" << to_string(speed) << endl << "机器人位姿：" << endl << temp->tran.x << endl << temp->tran.y << endl << temp->tran.z << temp->rpy.rx << endl << temp->rpy.ry << endl << temp->rpy.rz << endl;
        outFile.close();*/

        //Sleep(1000);
        //ret = demo.joint_move(&temp, MoveMode::ABS, false, 5, 400 * Deg2Rad, 2, NULL);
    }


    // 返回初始位置
    demo.joint_move(&inijval, MoveMode::ABS, true, 3.14,400*Deg2Rad,2,NULL);
    // Sleep(3000);
}

int main()
{
    std::thread t1(getVision);
    t1.detach(); // 分离线程，独立运行
    std::thread t2(getReferee);
    t2.detach(); // 分离线程，独立运行
    demo.login_in("192.168.1.112");
    demo.power_on();
    demo.enable_robot();

    
    //inijval.jVal[0] = 61.638 * Deg2Rad;
    //inijval.jVal[1] = 10.144 * Deg2Rad;
    //inijval.jVal[2] = 70.353 * Deg2Rad;
    //inijval.jVal[3] = -100.183 * Deg2Rad;
    //inijval.jVal[4] = -19.455 * Deg2Rad;
    //inijval.jVal[5] = -293.661 * Deg2Rad;

    inijval.jVal[0] = 0 * Deg2Rad;
    inijval.jVal[1] =0 * Deg2Rad;
    inijval.jVal[2] = 90 * Deg2Rad;
    inijval.jVal[3] = 0 * Deg2Rad;
    inijval.jVal[4] = 0 * Deg2Rad;
    inijval.jVal[5] = 0 * Deg2Rad;

    demo.joint_move(&inijval, MoveMode::ABS, true, 3);
    

    mkmotor.ID = 1;           // 电机ID 为1
    mkmotor.reduceRatio = 10; // 设置减速比
    mkmotor.setSerial(14, 115200);
    mkmotor.start();

    mkmotor.goToPos2(180, 1, 200, false);

    int round = 1;
    // 12轮击球，前2轮为试球，后10轮为正式比赛
    while (cmd != 'e')
    {
        /*cin >> cmd;*/
        if (cmd == 'k')
        {
            if (round == 1 || round == 2)
            {
                cout << "第" << round << "轮试球" << endl;
                
                std::cout << "calculate begin" << std::endl;
                calculate_hit();
                std::cout << "calculate end" << std::endl;
            }
            else
            {
                cout << "第" << round-2 << "轮正式比赛" << endl;
                
                std::cout << "calculate begin" << std::endl;
                calculate_hit();
                std::cout << "calculate end" << std::endl;
            }
            round++;
        }
    }
    return 0;
}

// 函数：持续接收视觉数据
void getVision()
{
    RoboCupSSLClient client;
    client.open(true); // 打开客户端连接
    SSL_WrapperPacket packet;
    std::vector<Point> ballP; // 存储球的位置历史记录

    while (true)
    {
        if (client.receive(packet))
        {
            if (packet.has_detection())
            { // 检查是否有检测到的数据
                SSL_DetectionFrame detection = packet.detection();
                int balls_n = detection.balls_size(); // 获取检测到的球的数量

                // 更新球的位置
                for (int i = 0; i < balls_n; i++)
                {
                    SSL_DetectionBall ball = detection.balls(i);
                    Point pp;
                    pp.x = ball.x() - 20; // 调整 x 位置，减去 20 个单位
                    pp.y = ball.y();
                    pp.t = (detection.t_capture() / 10 - (int)(detection.t_capture() / 10)); // 计算时间信息
                    mtx.lock();                                                              // 加锁以确保线程安全
                    ballPos = pp;                                                            // 更新全局的球位置
                    mtx.unlock();                                                            // 解锁

                    ballP.push_back(pp); // 将球的位置添加到历史记录中
                    if (pp.x < 300)      // 如果球的位置接近（x 坐标小于 300），记录数据
                    {
                        for (auto ball2 : ballP)
                        {
                            // outfile << ball2.x << "," << ball2.y << "," << ball2.t << '\n'; // 将球的位置和时间写入文件
                        }
                        // outfile.close(); // 关闭文件
                    }
                }
            }
        }
    }
    return;
}

double goAvalue(double ball_x, double ball_y, double posi_x, double posi_y, double angle)
{
    double run_x = 2740 - ball_x + posi_x;
    double run_y = ball_y - posi_y;
    double arctan = atan(run_y / run_x);
    // std::cout << arctan << std::endl;
    return arctan * Rad2Deg;
}

double speedX(double ball_x, double ball_y, double posi_x, double posi_y, double angle)
{

    // 最远点920
    // 左点700
    // 右点750

    // h=70cm
    double acc = 1.055;

    if (ball_y > 0)
    {
        if (ball_y < 200)
        {
            acc = 1.055;
        }
        else if (ball_y >= 200 && ball_y <= 400)
        {
            acc = 1.058;
        }
        else
        {
            acc = 1.055;
        }
    }
    else
    {
        if (ball_y > -200)
        {
            acc = 1.055;
        }
        else if (ball_y >= -400 && ball_y <= -200)
        {
            acc = 1.058;
        }
        else
        {
            acc = 1.055;
        }
    }

   //double acc = 1.058;

   // if (ball_y > 0)
   // {
   //     if (ball_y < 200)
   //     {
   //         acc = 1.057;
   //     }
   //     else if (ball_y >= 200 && ball_y <= 400)
   //     {
   //         acc = 1.060;
   //     }
   //     else
   //     {
   //         acc = 1.057;
   //     }
   // }
   // else
   // {
   //     if (ball_y > -200)
   //     {
   //         acc = 1.057;
   //     }
   //     else if (ball_y >= -400 && ball_y <= -200)
   //     {
   //         acc = 1.060;
   //     }
   //     else
   //     {
   //         acc = 1.057;
   //     }
   // }

    double t = sqrt(2 * 0.65 / 9.7967);
    double s = getdistance(ball_x, ball_y, posi_x, posi_y);
    cout << "s" << s << endl;
    double s1 = ((2740 - ball_x) / cos(goAvalue(ball_x, ball_y, posi_x, posi_y, angle) * Deg2Rad)) / acc;
    cout << "cos " << goAvalue(ball_x, ball_y, posi_x, posi_y, angle) * Deg2Rad << endl;
    cout << "s1" << s1 << endl;
    double dis = s - s1;
    cout << dis << endl;
    double v = dis / t;
    double ra = ((v / 1000) / 0.2) * Rad2Deg;
    if (ra > 900)
    {
        ra = 900;
    }

    return ra;
}

double getdistance(double ball_x, double ball_y, double posi_x, double posi_y)
{
    double a_posi_x = posi_x + 2740;

    double sqr = (ball_x - a_posi_x) * (ball_x - a_posi_x) + (ball_y - posi_y) * (ball_y - posi_y);
    return sqrt(sqr);
}

double get_hit_speed_direction(std::vector<Point> balls)
{

    if (balls.size() < 2)
    {
        std::cerr << "至少需要两个点来计算斜率。" << std::endl;
        return 0.0; // 或抛出异常
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xy = 0.0;
    double sum_xx = 0.0;

    int n = balls.size();

    for (const auto &point : balls)
    {
        sum_x += point.x;
        sum_y += point.y;
        sum_xy += point.x * point.y;
        sum_xx += point.x * point.x;
    }

    // 使用最小二乘法公式计算斜率 m
    double m = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);

    double angle_radians = std::atan(m);
    double angle_degrees = angle_radians * (180.0 / M_PI);
    return angle_degrees; // 返回以度为单位的角度
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
                hit_score.emplace(pair<int, int>(round, team.score()));
            }
        }
    }
    return;
}

//double speedX(double predict_x, double predict_y, int tong, std::vector<Point> ball) {
//    return 0;
//}




double angleBetweenLines(double m1, double m2) {
    return std::atan(fabs((m2 - m1) / (1 + m1 * m2))) * 180 / M_PI;
}

std::pair<double, double> fitLine(const std::vector<Point>& ball, double pre_x, double pre_y) {
    // 丢弃首尾各两个点 
    int startIndex =5;
    // // 从第三个点开始 
    int endIndex = ball.size() -5; 
    // // 到倒数第三个点为止 
    // // 使用中间的点和 (pre_x, pre_y)进行拟合 
    double sum_x =0, sum_y =0, sum_xy =0, sum_xx =0;  
    int n = 0;

    for (int i = startIndex; i < endIndex; ++i) {
        const auto& p = ball[i];
        sum_x += p.x;
        sum_y += p.y;
        sum_xy += p.x * p.y;
        sum_xx += p.x * p.x;
        n++;
    }

    // 加入 (pre_x, pre_y)  
    sum_x += pre_x;
    sum_y += pre_y;
    sum_xy += pre_x * pre_y;
    sum_xx += pre_x * pre_x;
    n++; //统计数量增加1 //计算斜率 m 和截距 b，防止分母为零
    double m =0.0, b =0.0;  
    if (n > 0 && (n * sum_xx - sum_x * sum_x) != 0) {
        m = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
        b = (sum_y - m * sum_x) / n;
    }

    return { m, b }; // 返回斜率和截距
}  

int choice(std::vector<Point> ball, double pre_x, double pre_y) {

    const double MAX_ALLOWED = 7;
        auto lineA = fitLine(ball, pre_x, pre_y);
        double mA = lineA.first;

        // 定义目标桶的位置 
        std::map<int, std::pair<double, double>> tong_position1 = {
     {1, {877.56 + 2740, 0}},
    {2, {640 + 2740, 250}},
    {3, {640 + 2740, -250}},
    {4, {278.88 + 2740, 563.88}},
    {5, {278.88 + 2740, -563.88}},
    {6, {278.88 + 2740, 0}}
        };



    //double minAngle = std::numeric_limits<double>::max();
    //int bestLineIndex = -1;

    //for (int i : {2, 3, 4, 5}) {
    //    auto pointB = tong_position1[i];

    //    //直线B的斜率 
    //    double mB = (pointB.second - pre_y) / (pointB.first - pre_x);  

    //    //计算夹角 
    //    double angle = angleBetweenLines(mA, mB);  
    //    cout << "第" << i << "夹角为 " << angle << endl;

    //    if (angle < minAngle) {
    //        minAngle = angle;
    //        bestLineIndex = i;
    //    }


    //}
    //if (minAngle < MAX_ALLOWED)
    //    cout << bestLineIndex << endl;
    //    return bestLineIndex; // 返回夹角最小的直线B的目标点序号
    //if (minAngle >= MAX_ALLOWED)
    return 6;
 }  