#include "im6014_motor.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <exception>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <mutex>
#include <sys/ioctl.h>
#include <sys/select.h>

namespace {

constexpr int TERMIOS2_NCCS = 19;
constexpr unsigned int TERMIOS2_CBAUD = 0x0000100F;
constexpr unsigned int TERMIOS2_BOTHER = 0x00001000;

struct termios2_linux {
    tcflag_t c_iflag;
    tcflag_t c_oflag;
    tcflag_t c_cflag;
    tcflag_t c_lflag;
    cc_t c_line;
    cc_t c_cc[TERMIOS2_NCCS];
    speed_t c_ispeed;
    speed_t c_ospeed;
};

#ifdef TCGETS2
#undef TCGETS2
#endif
#define TCGETS2 _IOR('T', 0x2A, termios2_linux)

#ifdef TCSETS2
#undef TCSETS2
#endif
#define TCSETS2 _IOW('T', 0x2B, termios2_linux)

constexpr double DEFAULT_GEAR_RATIO = 38.0 / 3.0;
constexpr double PI_VALUE = 3.14159265358979323846;
constexpr double MAX_OUTPUT_TORQUE_CMD = 162.128385;
constexpr double MAX_OUTPUT_SPEED_CMD = 253.965213;
constexpr double MAX_OUTPUT_POSITION_CMD = 32508.539376;
constexpr double MAX_OUTPUT_KP = 410.725243;
constexpr double MAX_OUTPUT_KD = 102.681311;

// Software zero offsets, reducer output side [rad].
// Edit this table for assembly calibration. Unlisted motors use offset = 0.
double configuredGearRatio(const std::string& port, int id) {
    if (port == "/dev/ttyUSB0") {
        switch (id) {
        case 0: return DEFAULT_GEAR_RATIO;
        case 1: return DEFAULT_GEAR_RATIO;
        case 2: return DEFAULT_GEAR_RATIO;
        case 3: return DEFAULT_GEAR_RATIO;
        default: return DEFAULT_GEAR_RATIO;
        }
    }
    if (port == "/dev/ttyUSB1") {
        switch (id) {
        case 0: return DEFAULT_GEAR_RATIO;
        case 1: return DEFAULT_GEAR_RATIO;
        case 2: return DEFAULT_GEAR_RATIO;
        case 3: return DEFAULT_GEAR_RATIO;
        default: return DEFAULT_GEAR_RATIO;
        }
    }
    if (port == "/dev/ttyUSB2") {
        switch (id) {
        case 0: return DEFAULT_GEAR_RATIO;
        case 1: return DEFAULT_GEAR_RATIO;
        case 2: return DEFAULT_GEAR_RATIO;
        case 3: return DEFAULT_GEAR_RATIO;
        default: return DEFAULT_GEAR_RATIO;
        }
    }
    if (port == "/dev/ttyUSB3") {
        switch (id) {
        case 0: return DEFAULT_GEAR_RATIO;
        case 1: return DEFAULT_GEAR_RATIO;
        case 2: return DEFAULT_GEAR_RATIO;
        case 3: return DEFAULT_GEAR_RATIO;
        default: return DEFAULT_GEAR_RATIO;
        }
    }
    return DEFAULT_GEAR_RATIO;
}

double configuredPositionOffset(const std::string& port, int id) {
    if (port == "/dev/ttyUSB0") {
        switch (id) {
        case 0: return 0.0;
        case 1: return 0.0;
        case 2: return 0.0;
        case 3: return 0.0;
        default: return 0.0;
        }
    }
    if (port == "/dev/ttyUSB1") {
        switch (id) {
        case 0: return 0.0;
        case 1: return 0.0;
        case 2: return 0.0;
        case 3: return 0.0;
        default: return 0.0;
        }
    }
    if (port == "/dev/ttyUSB2") {
        switch (id) {
        case 0: return 0.0;
        case 1: return 0.0;
        case 2: return 0.0;
        case 3: return 0.0;
        default: return 0.0;
        }
    }
    if (port == "/dev/ttyUSB3") {
        switch (id) {
        case 0: return 0.0;
        case 1: return 0.0;
        case 2: return 0.0;
        case 3: return 0.0;
        default: return 0.0;
        }
    }
    return 0.0;
}

const std::uint32_t CRC32_TABLE[256] = {
    0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B, 0x1A864DB2, 0x1E475005,
    0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61, 0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD,
    0x4C11DB70, 0x48D0C6C7, 0x4593E01E, 0x4152FDA9, 0x5F15ADAC, 0x5BD4B01B, 0x569796C2, 0x52568B75,
    0x6A1936C8, 0x6ED82B7F, 0x639B0DA6, 0x675A1011, 0x791D4014, 0x7DDC5DA3, 0x709F7B7A, 0x745E66CD,
    0x9823B6E0, 0x9CE2AB57, 0x91A18D8E, 0x95609039, 0x8B27C03C, 0x8FE6DD8B, 0x82A5FB52, 0x8664E6E5,
    0xBE2B5B58, 0xBAEA46EF, 0xB7A96036, 0xB3687D81, 0xAD2F2D84, 0xA9EE3033, 0xA4AD16EA, 0xA06C0B5D,
    0xD4326D90, 0xD0F37027, 0xDDB056FE, 0xD9714B49, 0xC7361B4C, 0xC3F706FB, 0xCEB42022, 0xCA753D95,
    0xF23A8028, 0xF6FB9D9F, 0xFBB8BB46, 0xFF79A6F1, 0xE13EF6F4, 0xE5FFEB43, 0xE8BCCD9A, 0xEC7DD02D,
    0x34867077, 0x30476DC0, 0x3D044B19, 0x39C556AE, 0x278206AB, 0x23431B1C, 0x2E003DC5, 0x2AC12072,
    0x128E9DCF, 0x164F8078, 0x1B0CA6A1, 0x1FCDBB16, 0x018AEB13, 0x054BF6A4, 0x0808D07D, 0x0CC9CDCA,
    0x7897AB07, 0x7C56B6B0, 0x71159069, 0x75D48DDE, 0x6B93DDDB, 0x6F52C06C, 0x6211E6B5, 0x66D0FB02,
    0x5E9F46BF, 0x5A5E5B08, 0x571D7DD1, 0x53DC6066, 0x4D9B3063, 0x495A2DD4, 0x44190B0D, 0x40D816BA,
    0xACA5C697, 0xA864DB20, 0xA527FDF9, 0xA1E6E04E, 0xBFA1B04B, 0xBB60ADFC, 0xB6238B25, 0xB2E29692,
    0x8AAD2B2F, 0x8E6C3698, 0x832F1041, 0x87EE0DF6, 0x99A95DF3, 0x9D684044, 0x902B669D, 0x94EA7B2A,
    0xE0B41DE7, 0xE4750050, 0xE9362689, 0xEDF73B3E, 0xF3B06B3B, 0xF771768C, 0xFA325055, 0xFEF34DE2,
    0xC6BCF05F, 0xC27DEDE8, 0xCF3ECB31, 0xCBFFD686, 0xD5B88683, 0xD1799B34, 0xDC3ABDED, 0xD8FBA05A,
    0x690CE0EE, 0x6DCDFD59, 0x608EDB80, 0x644FC637, 0x7A089632, 0x7EC98B85, 0x738AAD5C, 0x774BB0EB,
    0x4F040D56, 0x4BC510E1, 0x46863638, 0x42472B8F, 0x5C007B8A, 0x58C1663D, 0x558240E4, 0x51435D53,
    0x251D3B9E, 0x21DC2629, 0x2C9F00F0, 0x285E1D47, 0x36194D42, 0x32D850F5, 0x3F9B762C, 0x3B5A6B9B,
    0x0315D626, 0x07D4CB91, 0x0A97ED48, 0x0E56F0FF, 0x1011A0FA, 0x14D0BD4D, 0x19939B94, 0x1D528623,
    0xF12F560E, 0xF5EE4BB9, 0xF8AD6D60, 0xFC6C70D7, 0xE22B20D2, 0xE6EA3D65, 0xEBA91BBC, 0xEF68060B,
    0xD727BBB6, 0xD3E6A601, 0xDEA580D8, 0xDA649D6F, 0xC423CD6A, 0xC0E2D0DD, 0xCDA1F604, 0xC960EBB3,
    0xBD3E8D7E, 0xB9FF90C9, 0xB4BCB610, 0xB07DABA7, 0xAE3AFBA2, 0xAAFBE615, 0xA7B8C0CC, 0xA379DD7B,
    0x9B3660C6, 0x9FF77D71, 0x92B45BA8, 0x9675461F, 0x8832161A, 0x8CF30BAD, 0x81B02D74, 0x857130C3,
    0x5D8A9099, 0x594B8D2E, 0x5408ABF7, 0x50C9B640, 0x4E8EE645, 0x4A4FFBF2, 0x470CDD2B, 0x43CDC09C,
    0x7B827D21, 0x7F436096, 0x7200464F, 0x76C15BF8, 0x68860BFD, 0x6C47164A, 0x61043093, 0x65C52D24,
    0x119B4BE9, 0x155A565E, 0x18197087, 0x1CD86D30, 0x029F3D35, 0x065E2082, 0x0B1D065B, 0x0FDC1BEC,
    0x3793A651, 0x3352BBE6, 0x3E119D3F, 0x3AD08088, 0x2497D08D, 0x2056CD3A, 0x2D15EBE3, 0x29D4F654,
    0xC5A92679, 0xC1683BCE, 0xCC2B1D17, 0xC8EA00A0, 0xD6AD50A5, 0xD26C4D12, 0xDF2F6BCB, 0xDBEE767C,
    0xE3A1CBC1, 0xE760D676, 0xEA23F0AF, 0xEEE2ED18, 0xF0A5BD1D, 0xF464A0AA, 0xF9278673, 0xFDE69BC4,
    0x89B8FD09, 0x8D79E0BE, 0x803AC667, 0x84FBDBD0, 0x9ABC8BD5, 0x9E7D9662, 0x933EB0BB, 0x97FFAD0C,
    0xAFB010B1, 0xAB710D06, 0xA6322BDF, 0xA2F33668, 0xBCB4666D, 0xB8757BDA, 0xB5365D03, 0xB1F740B4,
};

std::uint32_t crc32Mpeg2DemoCompatible(const std::uint8_t* data, std::size_t len) {
    std::uint32_t crc = 0xFFFFFFFF;
    std::size_t i = 0;
    while (i + 3 < len) {
        const std::uint8_t bytes[4] = {data[i + 3], data[i + 2], data[i + 1], data[i]};
        for (std::uint8_t b : bytes) {
            crc = CRC32_TABLE[((crc >> 24) ^ b) & 0xFF] ^ (crc << 8);
        }
        i += 4;
    }
    return crc;
}

void putLe16(std::uint8_t* p, std::int16_t v) {
    p[0] = static_cast<std::uint8_t>(v & 0xFF);
    p[1] = static_cast<std::uint8_t>((v >> 8) & 0xFF);
}

void putLe32(std::uint8_t* p, std::int32_t v) {
    p[0] = static_cast<std::uint8_t>(v & 0xFF);
    p[1] = static_cast<std::uint8_t>((v >> 8) & 0xFF);
    p[2] = static_cast<std::uint8_t>((v >> 16) & 0xFF);
    p[3] = static_cast<std::uint8_t>((v >> 24) & 0xFF);
}

std::uint16_t getLe16u(const std::uint8_t* p) {
    return static_cast<std::uint16_t>(p[0] | (p[1] << 8));
}

std::int16_t getLe16s(const std::uint8_t* p) {
    return static_cast<std::int16_t>(getLe16u(p));
}

std::uint32_t getLe32u(const std::uint8_t* p) {
    return static_cast<std::uint32_t>(p[0])
        | (static_cast<std::uint32_t>(p[1]) << 8)
        | (static_cast<std::uint32_t>(p[2]) << 16)
        | (static_cast<std::uint32_t>(p[3]) << 24);
}

std::int32_t getLe32s(const std::uint8_t* p) {
    return static_cast<std::int32_t>(getLe32u(p));
}

bool baudrateToConstant(int baudrate, speed_t& speed) {
    switch (baudrate) {
    case 9600: speed = B9600; return true;
    case 19200: speed = B19200; return true;
    case 38400: speed = B38400; return true;
    case 57600: speed = B57600; return true;
    case 115200: speed = B115200; return true;
    case 230400: speed = B230400; return true;
    case 460800: speed = B460800; return true;
    case 500000: speed = B500000; return true;
    case 576000: speed = B576000; return true;
    case 921600: speed = B921600; return true;
    case 1000000: speed = B1000000; return true;
    case 1152000: speed = B1152000; return true;
    case 1500000: speed = B1500000; return true;
    case 2000000: speed = B2000000; return true;
    case 2500000: speed = B2500000; return true;
    case 3000000: speed = B3000000; return true;
    case 3500000: speed = B3500000; return true;
    case 4000000: speed = B4000000; return true;
#ifdef B6000000
    case 6000000: speed = B6000000; return true;
#endif
    default:
        return false;
    }
}

std::string errnoMessage(const std::string& prefix) {
    return prefix + ": " + std::strerror(errno);
}

void setCustomBaudrate(int fd, int baudrate) {
    termios2_linux tio {};
    if (ioctl(fd, TCGETS2, &tio) != 0) {
        throw std::runtime_error(errnoMessage("TCGETS2 failed for custom baudrate"));
    }

    tio.c_cflag &= ~TERMIOS2_CBAUD;
    tio.c_cflag |= TERMIOS2_BOTHER;
    tio.c_ispeed = static_cast<speed_t>(baudrate);
    tio.c_ospeed = static_cast<speed_t>(baudrate);

    if (ioctl(fd, TCSETS2, &tio) != 0) {
        throw std::runtime_error(errnoMessage("TCSETS2 failed for custom baudrate"));
    }
}

} // namespace

MotorController::MotorController(const std::string& port, int baudrate, double timeout_sec)
    : port_(port), timeout_sec_(timeout_sec) {
    openPort(port, baudrate);
}

MotorController::~MotorController() {
    close();
}

void MotorController::openPort(const std::string& port, int baudrate) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        throw std::runtime_error(errnoMessage("open serial port failed"));
    }

    termios tty {};
    if (tcgetattr(fd_, &tty) != 0) {
        close();
        throw std::runtime_error(errnoMessage("tcgetattr failed"));
    }

    cfmakeraw(&tty);
    speed_t speed = B38400;
    const bool has_standard_baudrate = baudrateToConstant(baudrate, speed);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag = static_cast<tcflag_t>((tty.c_cflag & ~CSIZE) | CS8);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= static_cast<tcflag_t>(~(PARENB | PARODD | CSTOPB | CRTSCTS));
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        close();
        throw std::runtime_error(errnoMessage("tcsetattr failed"));
    }

    if (!has_standard_baudrate) {
        try {
            setCustomBaudrate(fd_, baudrate);
        } catch (...) {
            close();
            throw;
        }
    }
}

void MotorController::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool MotorController::isOpen() const {
    return fd_ >= 0;
}

void MotorController::validateId(int id) {
    if (id < 0 || id > 14) {
        throw std::invalid_argument("motor id must be 0~14; id=15 is broadcast and has no feedback");
    }
}

MotorController::Command MotorController::makeCommand(int id,
                                                    int mode,
                                                    int timeout,
                                                    double tor,
                                                    double spd,
                                                    double pos,
                                                    double kp,
                                                    double kd) {
    validateId(id);
    if (mode != 0 && mode != 1) {
        throw std::invalid_argument("mode must be 0 (stop) or 1 (FOC)");
    }

    Command cmd;
    cmd.mode = mode;
    cmd.timeout = timeout ? 1 : 0;
    cmd.tor = tor;
    cmd.spd = spd;
    cmd.pos = pos + configuredPositionOffset(port_, id);
    cmd.kp = kp;
    cmd.kd = kd;
    last_command_[id] = cmd;
    return cmd;
}

MotorStatus MotorController::positionControl(int id,
                                             int mode,
                                             int timeout,
                                             double tor,
                                             double spd,
                                             double pos,
                                             double kp,
                                             double kd) {
    const Command cmd = makeCommand(id, mode, timeout, tor, spd, pos, kp, kd);
    return sendCommand(id, cmd);
}

MotorStatus MotorController::readStatus(int id) {
    validateId(id);
    auto it = last_command_.find(id);
    if (it != last_command_.end()) {
        return sendCommand(id, it->second);
    }
    return sendCommand(id, Command {});
}

MotorStatus MotorController::stop(int id, int timeout) {
    return positionControl(id, 0, timeout, 0.0, 0.0, 0.0, 0.0, 0.0);
}

MotorStatus MotorController::setVelocity(int id,
                                         double target_speed,
                                         const VelocityControlOptions& options) {
    return positionControl(
        id,
        1,
        options.timeout_protection,
        options.torque,
        target_speed,
        0.0,
        0.0,
        options.kd
    );
}

PositionMoveResult MotorController::moveToPosition(int id,
                                                   double target_pos,
                                                   const PositionMoveOptions& options) {
    MotorStatus status = positionControl(
        id,
        1,
        options.timeout_protection,
        options.torque,
        options.speed,
        target_pos,
        options.kp,
        options.kd
    );

    const auto start = std::chrono::steady_clock::now();
    bool reached = false;

    while (true) {
        const double pos_error = std::abs(status.position - target_pos);
        const double abs_speed = std::abs(status.speed);

        if (pos_error <= options.position_tolerance && abs_speed <= options.speed_tolerance) {
            reached = true;
            break;
        }

        if (std::chrono::steady_clock::now() - start >= options.max_wait) {
            break;
        }

        std::this_thread::sleep_for(options.query_period);
        status = readStatus(id);
    }

    if (options.stop_after_move) {
        status = stop(id);
    }

    return PositionMoveResult {status, reached};
}

MotorStatus MotorController::sendCommand(int id, const Command& cmd) {
    if (fd_ < 0) {
        throw std::runtime_error("serial port is not open");
    }

    std::uint8_t packet[20] {};
    buildControlPacket(id, cmd, packet);

    tcflush(fd_, TCIFLUSH);
    const ssize_t written = ::write(fd_, packet, sizeof(packet));
    if (written != static_cast<ssize_t>(sizeof(packet))) {
        throw std::runtime_error(errnoMessage("incomplete serial write"));
    }
    tcdrain(fd_);

    std::uint8_t feedback[26] {};
    std::size_t received = 0;
    while (received < sizeof(feedback)) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd_, &readfds);

        timeval tv {};
        tv.tv_sec = static_cast<long>(timeout_sec_);
        tv.tv_usec = static_cast<long>((timeout_sec_ - tv.tv_sec) * 1000000.0);

        const int ready = select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
        if (ready < 0) {
            throw std::runtime_error(errnoMessage("select failed"));
        }
        if (ready == 0) {
            throw std::runtime_error("motor " + std::to_string(id) + ": expected 26 feedback bytes, got "
                                     + std::to_string(received));
        }

        const ssize_t n = ::read(fd_, feedback + received, sizeof(feedback) - received);
        if (n < 0) {
            throw std::runtime_error(errnoMessage("serial read failed"));
        }
        if (n == 0) {
            continue;
        }
        received += static_cast<std::size_t>(n);
    }

    return parseFeedbackPacket(feedback);
}

void MotorController::buildControlPacket(int id, const Command& cmd, std::uint8_t packet[20]) const {
    const double ratio = configuredGearRatio(port_, id);
    const double kp = std::clamp(cmd.kp, 0.0, MAX_OUTPUT_KP);
    const double kd = std::clamp(cmd.kd, 0.0, MAX_OUTPUT_KD);
    const double tor = std::clamp(cmd.tor, -MAX_OUTPUT_TORQUE_CMD, MAX_OUTPUT_TORQUE_CMD);
    const double spd = std::clamp(cmd.spd, -MAX_OUTPUT_SPEED_CMD, MAX_OUTPUT_SPEED_CMD);
    const double pos = std::clamp(cmd.pos, -MAX_OUTPUT_POSITION_CMD, MAX_OUTPUT_POSITION_CMD);

    int k_pos_val = static_cast<int>(std::llround(kp / (ratio * ratio) * 12800.0));
    int k_spd_val = static_cast<int>(std::llround(kd / (ratio * ratio) * 51200.0));
    int pos_des_val = static_cast<int>(std::llround(pos * ratio * 32768.0 / (2.0 * PI_VALUE)));
    int spd_des_val = static_cast<int>(std::llround(spd * ratio * 64.0 / (2.0 * PI_VALUE)));
    int tor_des_val = static_cast<int>(std::llround(tor / ratio * 2560.0));

    k_pos_val = std::clamp(k_pos_val, 0, 32767);
    k_spd_val = std::clamp(k_spd_val, 0, 32767);
    spd_des_val = std::clamp(spd_des_val, -32768, 32767);
    tor_des_val = std::clamp(tor_des_val, -32768, 32767);

    packet[0] = 0xFE;
    packet[1] = 0xEE;
    packet[2] = static_cast<std::uint8_t>((id & 0x0F) | ((cmd.mode & 0x07) << 4) | ((cmd.timeout & 0x01) << 7));
    packet[3] = 0x00;
    putLe16(packet + 4, static_cast<std::int16_t>(tor_des_val));
    putLe16(packet + 6, static_cast<std::int16_t>(spd_des_val));
    putLe32(packet + 8, static_cast<std::int32_t>(pos_des_val));
    putLe16(packet + 12, static_cast<std::int16_t>(k_pos_val));
    putLe16(packet + 14, static_cast<std::int16_t>(k_spd_val));

    const std::uint32_t crc = crc32Mpeg2DemoCompatible(packet, 16);
    putLe32(packet + 16, static_cast<std::int32_t>(crc));
}

MotorStatus MotorController::parseFeedbackPacket(const std::uint8_t data[26]) const {
    if (data[0] != 0xFC || data[1] != 0xEE) {
        throw std::runtime_error("invalid feedback header");
    }

    const std::uint32_t crc_received = getLe32u(data + 22);
    const std::uint32_t crc_computed = crc32Mpeg2DemoCompatible(data + 2, 20);
    if (crc_received != crc_computed) {
        throw std::runtime_error("feedback CRC mismatch");
    }

    const std::uint8_t mode_byte = data[2];
    const std::int8_t temp1 = static_cast<std::int8_t>(data[3]);
    const std::uint8_t temp2 = data[4];
    const std::uint8_t vol_raw = data[5];
    const std::int16_t torque_raw = getLe16s(data + 6);
    const std::int16_t speed_raw = getLe16s(data + 8);
    const std::int32_t pos_raw = getLe32s(data + 10);
    const std::uint32_t motor_error = getLe32u(data + 14);
    const std::uint16_t res_warning = getLe16u(data + 18);

    const double ratio = configuredGearRatio(port_, mode_byte & 0x0F);
    const double rotor_torque = torque_raw / 2560.0;
    const double rotor_speed = speed_raw * (2.0 * PI_VALUE) / 64.0;
    const double rotor_position = pos_raw * (2.0 * PI_VALUE) / 32768.0;

    MotorStatus status;
    status.id = mode_byte & 0x0F;
    status.mode = (mode_byte >> 4) & 0x07;
    status.timeout = (mode_byte >> 7) & 0x01;
    status.position = rotor_position / ratio - configuredPositionOffset(port_, status.id);
    status.speed = rotor_speed / ratio;
    status.torque = rotor_torque * ratio;
    status.voltage = vol_raw / 2.0;
    status.driver_temp = temp1;
    status.winding_temp = temp2;
    status.error = motor_error;
    status.warning = (res_warning >> 13) & 0x07;
    return status;
}

MotorEnableMask makeAllMotorsEnabled() {
    MotorEnableMask mask {};
    for (auto& bus : mask) {
        bus.fill(true);
    }
    return mask;
}

MotorEnableMask makeNoMotorsEnabled() {
    MotorEnableMask mask {};
    for (auto& bus : mask) {
        bus.fill(false);
    }
    return mask;
}

WheelLegRobot::WheelLegRobot(const std::array<std::string, kBusCount>& ports,
                             int baudrate,
                             double timeout_sec) {
    for (int i = 0; i < kBusCount; ++i) {
        buses_[i] = std::make_unique<MotorController>(ports[i], baudrate, timeout_sec);
    }
}

void WheelLegRobot::validateBusIndex(int bus_index) {
    if (bus_index < 0 || bus_index >= kBusCount) {
        throw std::out_of_range("bus index must be 0~3");
    }
}

BusFeedback WheelLegRobot::sendBusCommand(int bus_index,
                                          const BusCommand& command,
                                          const PositionMoveOptions& position_options,
                                          const VelocityControlOptions& velocity_options) {
    return sendBusCommand(bus_index, command, {true, true, true, true}, position_options, velocity_options);
}

BusFeedback WheelLegRobot::sendBusCommand(int bus_index,
                                          const BusCommand& command,
                                          const std::array<bool, kMotorsPerBus>& enabled_motors,
                                          const PositionMoveOptions& position_options,
                                          const VelocityControlOptions& velocity_options) {
    validateBusIndex(bus_index);

    BusFeedback feedback;
    MotorController& bus = *buses_[bus_index];

    for (int id = 0; id < kPositionMotorCount; ++id) {
        if (!enabled_motors[id]) {
            continue;
        }

        feedback.motors[id] = bus.positionControl(
            id,
            1,
            position_options.timeout_protection,
            position_options.torque,
            position_options.speed,
            command.joint_positions[id],
            position_options.kp,
            position_options.kd
        );
    }

    if (enabled_motors[kWheelMotorId]) {
        feedback.motors[kWheelMotorId] = bus.setVelocity(
            kWheelMotorId,
            command.wheel_speed,
            velocity_options
        );
    }

    return feedback;
}

std::array<BusFeedback, WheelLegRobot::kBusCount> WheelLegRobot::sendCommand(
    const std::array<BusCommand, kBusCount>& commands,
    const PositionMoveOptions& position_options,
    const VelocityControlOptions& velocity_options) {
    return sendCommand(commands, makeAllMotorsEnabled(), position_options, velocity_options);
}

std::array<BusFeedback, WheelLegRobot::kBusCount> WheelLegRobot::sendCommand(
    const std::array<BusCommand, kBusCount>& commands,
    const MotorEnableMask& enabled_motors,
    const PositionMoveOptions& position_options,
    const VelocityControlOptions& velocity_options) {
    std::array<BusFeedback, kBusCount> feedback;
    std::array<std::exception_ptr, kBusCount> errors {};
    std::array<std::thread, kBusCount> workers;

    for (int bus = 0; bus < kBusCount; ++bus) {
        workers[bus] = std::thread([&, bus]() {
            try {
                feedback[bus] = sendBusCommand(bus, commands[bus], enabled_motors[bus], position_options, velocity_options);
            } catch (...) {
                errors[bus] = std::current_exception();
            }
        });
    }

    for (auto& worker : workers) {
        worker.join();
    }

    for (const auto& error : errors) {
        if (error) {
            std::rethrow_exception(error);
        }
    }

    return feedback;
}

BusFeedback WheelLegRobot::readBusStatus(int bus_index) {
    return readBusStatus(bus_index, {true, true, true, true});
}

BusFeedback WheelLegRobot::readBusStatus(int bus_index, const std::array<bool, kMotorsPerBus>& enabled_motors) {
    validateBusIndex(bus_index);

    BusFeedback feedback;
    MotorController& bus = *buses_[bus_index];
    for (int id = 0; id < kMotorsPerBus; ++id) {
        if (enabled_motors[id]) {
            feedback.motors[id] = bus.readStatus(id);
        }
    }
    return feedback;
}

std::array<BusFeedback, WheelLegRobot::kBusCount> WheelLegRobot::readStatus() {
    return readStatus(makeAllMotorsEnabled());
}

std::array<BusFeedback, WheelLegRobot::kBusCount> WheelLegRobot::readStatus(const MotorEnableMask& enabled_motors) {
    std::array<BusFeedback, kBusCount> feedback;
    std::array<std::exception_ptr, kBusCount> errors {};
    std::array<std::thread, kBusCount> workers;

    for (int bus = 0; bus < kBusCount; ++bus) {
        workers[bus] = std::thread([&, bus]() {
            try {
                feedback[bus] = readBusStatus(bus, enabled_motors[bus]);
            } catch (...) {
                errors[bus] = std::current_exception();
            }
        });
    }

    for (auto& worker : workers) {
        worker.join();
    }

    for (const auto& error : errors) {
        if (error) {
            std::rethrow_exception(error);
        }
    }

    return feedback;
}

void WheelLegRobot::stopAll() {
    stopAll(makeAllMotorsEnabled());
}

void WheelLegRobot::stopAll(const MotorEnableMask& enabled_motors) {
    std::array<std::thread, kBusCount> workers;

    for (int bus = 0; bus < kBusCount; ++bus) {
        workers[bus] = std::thread([&, bus]() {
            for (int id = 0; id < kMotorsPerBus; ++id) {
                if (!enabled_motors[bus][id]) {
                    continue;
                }

                try {
                    buses_[bus]->stop(id);
                } catch (...) {
                    // stopAll is best-effort so one disconnected motor does not prevent others from stopping.
                }
            }
        });
    }

    for (auto& worker : workers) {
        worker.join();
    }
}

namespace im6014_simple {
namespace {

std::mutex& controllerMutex() {
    static std::mutex mutex;
    return mutex;
}

std::map<std::string, std::unique_ptr<MotorController>>& controllers() {
    static std::map<std::string, std::unique_ptr<MotorController>> instances;
    return instances;
}

MotorController& controllerFor(const std::string& port) {
    std::lock_guard<std::mutex> lock(controllerMutex());
    auto& instance = controllers()[port];
    if (!instance) {
        instance = std::make_unique<MotorController>(port);
    }
    return *instance;
}

} // namespace

MotorStatus enable(const std::string& port, int id) {
    return controllerFor(port).positionControl(id, 1, 1, 0.0, 0.0, 0.0, 0.0, 0.0);
}

MotorStatus positionControl(const std::string& port,
                            int id,
                            double feedforward_torque,
                            double kp,
                            double target_position,
                            double kd,
                            double target_speed) {
    return controllerFor(port).positionControl(
        id,
        1,
        1,
        feedforward_torque,
        target_speed,
        target_position,
        kp,
        kd
    );
}

MotorStatus velocityControl(const std::string& port,
                            int id,
                            double feedforward_torque,
                            double kd,
                            double target_speed) {
    VelocityControlOptions options;
    options.torque = feedforward_torque;
    options.kd = kd;
    options.timeout_protection = 1;
    return controllerFor(port).setVelocity(id, target_speed, options);
}

MotorStatus velocityControl(const std::string& port,
                            int id,
                            double feedforward_torque,
                            double kd) {
    return velocityControl(port, id, feedforward_torque, kd, 0.0);
}

MotorStatus readStatus(const std::string& port, int id) {
    return controllerFor(port).readStatus(id);
}

MotorStatus stop(const std::string& port, int id) {
    return controllerFor(port).stop(id);
}

void close(const std::string& port) {
    std::lock_guard<std::mutex> lock(controllerMutex());
    controllers().erase(port);
}

void closeAll() {
    std::lock_guard<std::mutex> lock(controllerMutex());
    controllers().clear();
}

} // namespace im6014_simple

