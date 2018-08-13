//
//  SerialDump
//

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <fcntl.h>
#include <memory.h>
#ifdef _WIN32
#  include <Windows.h>
#else
#  include <termios.h>
#  include <unistd.h>
#endif

// Silicon Labs CEL EM3588 USB Stick
//static const uint16_t CEL_VID = 0x10C4;
//static const uint16_t CEL_PID = 0x8A5E;
static const speed_t CEL_BAUD = B57600;
static const bool CEL_XON_XOFF = true;
static const bool CEL_RTS_CTS = false;

// Silicon Labs WSTK (J-Link Pro OB)
//static const uint16_t WSTK_VID = 0x1366;
//static const uint16_t WSTK_PID = 0x0105;
static const speed_t WSTK_BAUD = B115200;
static const bool WSTK_XON_XOFF = false;
static const bool WSTK_RTS_CTS = true;

class Device final
{
public:
    explicit Device(const std::string& path,
                    speed_t baud,
                    bool xOnOff,
                    bool rtsCts)
    {
        fd = open(path.c_str(), O_RDWR | O_NOCTTY);

        if (fd == -1) throw std::runtime_error("Failed to open " + path);

        struct termios settings;
        if (tcgetattr(fd, &settings) != 0)
            throw std::runtime_error("Failed to get attributes");

        if (cfsetospeed(&settings, baud) != 0) // baud rate
            throw std::runtime_error("Failed to set baud rate");
        if (cfsetispeed(&settings, baud) !=0)
            throw std::runtime_error("Failed to set baud rate");

        settings.c_cflag &= ~PARENB; // no parity
        settings.c_cflag &= ~CSTOPB; // 1 stop bit
        settings.c_cflag &= ~CSIZE;
        settings.c_cflag |= CS8 | CLOCAL; // 8 bits
        //settings.c_lflag = ICANON; // canonical mode
        //settings.c_oflag &= ~OPOST; // raw output

        // hardware flow control
        if (rtsCts) settings.c_cflag |= CRTSCTS;
        // software flow control
        if (xOnOff) settings.c_iflag |= IXON | IXOFF | IXANY;

        if (tcsetattr(fd, TCSANOW, &settings) != 0) // apply the settings
            throw std::runtime_error("Failed to set attributes");

        usleep(200000);
        tcflush(fd,TCIOFLUSH);
    }

    ~Device()
    {
        if (fd != -1) close(fd);
    }

    Device(const Device&) = delete;
    Device(Device&&) = delete;
    Device& operator=(const Device&) = delete;
    Device& operator=(Device&&) = delete;

    void sendData(const std::vector<uint8_t>& data)
    {
        printf("Sending:");
        for (uint8_t c : data)
            printf(" %x", c);
        printf("\n");

        int remaining = static_cast<int>(data.size());
        const uint8_t* ptr = static_cast<const uint8_t*>(data.data());

        while (remaining)
        {
            ssize_t result = write(fd, ptr, remaining);

            if (result == -1) throw std::runtime_error("Failed to write data");

            ptr += result;
            remaining -= result;
        }
    }

    std::vector<uint8_t> readData()
    {
        uint8_t buffer[256];
        std::vector<uint8_t> data;

        ssize_t result = read(fd, buffer, sizeof(buffer));

        if (result == -1) throw std::runtime_error("Failed to read data");
        else if (result == 0)
        {
            close(fd);
            fd = -1;
            throw std::runtime_error("Connection closed");
        }

        data.assign(std::begin(buffer), std::end(buffer));

        printf("Received:");
        for (uint8_t c : data)
            printf(" %x", c);
        printf("\n");

        return data;
    }

    bool isClosed() const
    {
#ifdef _WIN32
        return handle == INVALID_HANDLE_VALUE;
#else
        return fd == -1;
#endif
    }

private:
#ifdef _WIN32
    HANDLE handle = INVALID_HANDLE_VALUE;
#else
    int fd = -1;
#endif
};

int main(int argc, const char * argv[])
{
    try
    {
        std::string path;
        speed_t baud = CEL_BAUD;
        bool xOnOff = CEL_XON_XOFF;
        bool rtsCts = CEL_RTS_CTS;

        for (int i = 1; i < argc; ++i)
        {
            if (std::string(argv[i]) == "--path")
            {
                if (++i >= argc) throw std::runtime_error("Invalid parameter");
                path = argv[i];
            }
            else if (std::string(argv[i]) == "--device")
            {
                if (++i >= argc) throw std::runtime_error("Invalid parameter");

                if (std::string(argv[i]) == "CEL")
                {
                    baud = CEL_BAUD;
                    xOnOff = CEL_XON_XOFF;
                    rtsCts = CEL_RTS_CTS;
                }
                else if (std::string(argv[i]) == "WSTK")
                {
                    baud = WSTK_BAUD;
                    xOnOff = WSTK_XON_XOFF;
                    rtsCts = WSTK_RTS_CTS;
                }
                else
                    throw std::runtime_error("Invalid device");
            }
            else
                throw std::runtime_error("Invalid parameter");
        }

        if (path.empty()) throw std::runtime_error("No path");

        Device device(path, baud, xOnOff, rtsCts);

        while (!device.isClosed())
            device.readData();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (...)
    {
        std::cerr << "Error" << std::endl;
        return EXIT_FAILURE;
    }
}
