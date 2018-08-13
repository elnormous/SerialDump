//
//  SerialDump
//

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <fcntl.h>
#include <memory.h>
#include <termios.h>
#include <unistd.h>

// Silicon Labs CEL EM3588 USB Stick
static const uint16_t CEL_VID = 0x10C4;
static const uint16_t CEL_PID = 0x8A5E;
static const speed_t CEL_BAUD = B57600;
static const bool CEL_XON_XOFF = true;
static const bool CEL_RTS_CTS = false;

// Silicon Labs WSTK (J-Link Pro OB)
static const uint16_t WSTK_VID = 0x1366;
static const uint16_t WSTK_PID = 0x0105;
static const speed_t WSTK_BAUD = B115200;
static const bool WSTK_XON_XOFF = false;
static const bool WSTK_RTS_CTS = true;

/*
static const uint16_t VID = CEL_VID;
static const uint16_t PID = CEL_PID;
static const speed_t BAUD = CEL_BAUD;
static const bool XON_XOFF = CEL_XON_XOFF;
static const bool RTS_CTS = CEL_RTS_CTS;
/*/
static const uint16_t VID = WSTK_VID;
static const uint16_t PID = WSTK_PID;
static const speed_t BAUD = WSTK_BAUD;
static const bool XON_XOFF = WSTK_XON_XOFF;
static const bool RTS_CTS = WSTK_RTS_CTS;
//*/

class Device final
{
public:
    explicit Device(const std::string& path)
    {
        fd = open(path.c_str(), O_RDWR | O_NOCTTY);

        if (fd == -1) throw std::runtime_error("Failed to open " + path);

        struct termios settings;
        if (tcgetattr(fd, &settings) != 0)
            throw std::runtime_error("Failed to get attributes");

        const speed_t baud = BAUD; // baud rate

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
        if (RTS_CTS) settings.c_cflag |= CRTSCTS;
        // software flow control
        if (XON_XOFF) settings.c_iflag |= IXON | IXOFF | IXANY;

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

        int remaining = data.size();
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

    bool isClosed() const { return fd == -1; }

private:
    int fd = -1;
};

int main(int argc, const char * argv[])
{
    try
    {
        std::string path;

        for (int i = 1; i < argc; ++i)
        {
            if (std::string(argv[i]) == "--path")
            {
                if (++i >= argc) throw std::runtime_error("Invalid parameter");
                path = argv[i];
            }
        }

        if (path.empty()) throw std::runtime_error("No path");

        Device device(path);

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
