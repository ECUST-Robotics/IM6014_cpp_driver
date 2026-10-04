#include "im6014_motor.h"
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

volatile std::sig_atomic_t running = 1;
void handleSignal(int) {
    running = 0;
}

int main() {

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    try {

        // ①使能电机示例:（串口，电机ID）
        im6014_simple::enable("/dev/ttyUSB3", 1);

        while (running) {
            // ②位置控制示例:（串口，电机ID，前馈力矩，Kp，目标位置，Kd，目标速度）
            im6014_simple::positionControl("/dev/ttyUSB3", 1, 0.0, 20.0, 0.0, 0.1, 0.0);

            // ③速度控制示例:（串口，电机ID，前馈力矩，Kd，目标速度）最大速度：54.2rad/s
            // im6014_simple::velocityControl("/dev/ttyUSB3", 1, 0.2, 10.0, 6.24);

            // ④读取电机状态示例:（串口，电机ID）
            MotorStatus status = im6014_simple::readStatus("/dev/ttyUSB3", 1);
            std::cout << "pos=" << status.position
                      << " rad, spd=" << status.speed
                      << " rad/s, tor=" << status.torque
                      << " N*m, voltage=" << status.voltage << " V\n";


            // 控制周期为10ms，建议在循环中加入适当的延时，避免过于频繁的控制命令发送
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // ⑥停止电机并关闭串口示例:（串口，电机ID）
        im6014_simple::stop("/dev/ttyUSB3", 1);
        im6014_simple::closeAll();

    } catch (const std::exception& e) {
        std::cerr << "Motor test error: " << e.what() << '\n';
        im6014_simple::stop("/dev/ttyUSB3", 1);
        im6014_simple::closeAll();
        return 1;
    }

    return 0;
}
