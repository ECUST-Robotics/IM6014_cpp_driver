#ifndef IM6014_MOTOR_H
#define IM6014_MOTOR_H

#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

struct MotorStatus {
    int id = 0;
    int mode = 0;
    int timeout = 0;
    double position = 0.0;     // reducer output side, rad
    double speed = 0.0;        // reducer output side, rad/s
    double torque = 0.0;       // reducer output side, N*m
    double voltage = 0.0;
    int driver_temp = 0;
    int winding_temp = 0;
    std::uint32_t error = 0;
    int warning = 0;
};

struct PositionMoveOptions {
    double torque = 0.0;
    double speed = 0.0;
    double kp = 20.0;
    double kd = 0.1;
    double position_tolerance = 0.01;
    double speed_tolerance = 0.02;
    int timeout_protection = 1;
    bool stop_after_move = true;
    std::chrono::milliseconds max_wait {3000};
    std::chrono::milliseconds query_period {10};
};

struct PositionMoveResult {
    MotorStatus status;
    bool reached = false;
};

struct VelocityControlOptions {
    double torque = 0.2; // N*m, reducer output side
    double kd = 10.0;    // N*m/(rad/s), reducer output side
    int timeout_protection = 1;
};

struct BusCommand {
    std::array<double, 3> joint_positions {0.0, 0.0, 0.0}; // motor id 0,1,2 position targets, rad
    double wheel_speed = 0.0;                              // motor id 3 speed target, rad/s
};

struct BusFeedback {
    std::array<MotorStatus, 4> motors;
};

using MotorEnableMask = std::array<std::array<bool, 4>, 4>;

MotorEnableMask makeAllMotorsEnabled();
MotorEnableMask makeNoMotorsEnabled();

class MotorController {
public:
    explicit MotorController(const std::string& port,
                             int baudrate = 6000000,
                             double timeout_sec = 0.2);
    ~MotorController();

    MotorController(const MotorController&) = delete;
    MotorController& operator=(const MotorController&) = delete;

    MotorStatus positionControl(int id,
                                int mode,
                                int timeout,
                                double tor,
                                double spd,
                                double pos,
                                double kp,
                                double kd);
    MotorStatus readStatus(int id);
    MotorStatus stop(int id, int timeout = 0);
    MotorStatus setVelocity(int id,
                            double target_speed,
                            const VelocityControlOptions& options = VelocityControlOptions());
    PositionMoveResult moveToPosition(int id,
                                      double target_pos,
                                      const PositionMoveOptions& options = PositionMoveOptions());

    void close();
    bool isOpen() const;

private:
    struct Command {
        int mode = 0;
        int timeout = 0;
        double tor = 0.0;
        double spd = 0.0;
        double pos = 0.0;
        double kp = 0.0;
        double kd = 0.0;
    };

    int fd_ = -1;
    std::string port_;
    double timeout_sec_ = 0.2;
    std::map<int, Command> last_command_;

    static void validateId(int id);
    void openPort(const std::string& port, int baudrate);
    MotorStatus sendCommand(int id, const Command& cmd);
    Command makeCommand(int id, int mode, int timeout, double tor, double spd, double pos, double kp, double kd);
    void buildControlPacket(int id, const Command& cmd, std::uint8_t packet[20]) const;
    MotorStatus parseFeedbackPacket(const std::uint8_t data[26]) const;
};

class WheelLegRobot {
public:
    static constexpr int kBusCount = 4;
    static constexpr int kMotorsPerBus = 4;
    static constexpr int kPositionMotorCount = 3;
    static constexpr int kWheelMotorId = 3;

    explicit WheelLegRobot(const std::array<std::string, kBusCount>& ports,
                           int baudrate = 6000000,
                           double timeout_sec = 0.2);

    BusFeedback sendBusCommand(int bus_index,
                               const BusCommand& command,
                               const PositionMoveOptions& position_options = PositionMoveOptions(),
                               const VelocityControlOptions& velocity_options = VelocityControlOptions());
    BusFeedback sendBusCommand(int bus_index,
                               const BusCommand& command,
                               const std::array<bool, kMotorsPerBus>& enabled_motors,
                               const PositionMoveOptions& position_options = PositionMoveOptions(),
                               const VelocityControlOptions& velocity_options = VelocityControlOptions());

    std::array<BusFeedback, kBusCount> sendCommand(
        const std::array<BusCommand, kBusCount>& commands,
        const PositionMoveOptions& position_options = PositionMoveOptions(),
        const VelocityControlOptions& velocity_options = VelocityControlOptions());
    std::array<BusFeedback, kBusCount> sendCommand(
        const std::array<BusCommand, kBusCount>& commands,
        const MotorEnableMask& enabled_motors,
        const PositionMoveOptions& position_options = PositionMoveOptions(),
        const VelocityControlOptions& velocity_options = VelocityControlOptions());

    BusFeedback readBusStatus(int bus_index);
    BusFeedback readBusStatus(int bus_index, const std::array<bool, kMotorsPerBus>& enabled_motors);
    std::array<BusFeedback, kBusCount> readStatus();
    std::array<BusFeedback, kBusCount> readStatus(const MotorEnableMask& enabled_motors);

    void stopAll();
    void stopAll(const MotorEnableMask& enabled_motors);

private:
    std::array<std::unique_ptr<MotorController>, kBusCount> buses_;

    static void validateBusIndex(int bus_index);
};

namespace im6014_simple {

MotorStatus enable(const std::string& port, int id);
MotorStatus positionControl(const std::string& port,
                            int id,
                            double feedforward_torque,
                            double kp,
                            double target_position,
                            double kd,
                            double target_speed);
MotorStatus velocityControl(const std::string& port,
                            int id,
                            double feedforward_torque,
                            double kd,
                            double target_speed);
MotorStatus velocityControl(const std::string& port,
                            int id,
                            double feedforward_torque,
                            double kd);
MotorStatus readStatus(const std::string& port, int id);
MotorStatus stop(const std::string& port, int id);
void close(const std::string& port);
void closeAll();

} // namespace im6014_simple

#endif
