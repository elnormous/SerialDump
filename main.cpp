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
static const uint32_t CEL_BAUD = 57600;
static const bool CEL_XON_XOFF = true;
static const bool CEL_RTS_CTS = false;

// Silicon Labs WSTK (J-Link Pro OB)
//static const uint16_t WSTK_VID = 0x1366;
//static const uint16_t WSTK_PID = 0x0105;
static const uint32_t WSTK_BAUD = 115200;
static const bool WSTK_XON_XOFF = false;
static const bool WSTK_RTS_CTS = true;

class Device final
{
public:
    explicit Device(const std::string& path,
                    uint32_t baud,
                    bool xOnOff,
                    bool rtsCts)
    {
#ifdef _WIN32
        int size = MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, nullptr, 0);
        if (size == 0)
            throw std::runtime_error("Failed to convert UTF-8 to wide char");

        std::vector<WCHAR> buffer(size);
        if (MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, buffer.data(), size) == 0)
            throw std::runtime_error("Failed to convert the filename to wide char");

        // relative paths longer than MAX_PATH are not supported
        if (buffer.size() > MAX_PATH)
            buffer.insert(buffer.begin(), { L'\\', L'\\', L'?', L'\\' });

        handle = CreateFile(buffer.data(), GENERIC_READ | GENERIC_WRITE, 0,
                            nullptr, OPEN_EXISTING, 0, nullptr);

        if (handle == INVALID_HANDLE_VALUE)
            throw std::runtime_error("Failed to open " + path);

        DCB serialParameters;
        serialParameters.DCBlength = sizeof(serialParameters);

        if (!GetCommState(handle, &serialParameters))
            throw std::runtime_error("Failed to get parameters");

        DWORD speed;

        switch (baud)
        {
            case 110: speed = CBR_110; break;
            case 300: speed = CBR_300; break;
            case 600: speed = CBR_600; break;
            case 1200: speed = CBR_1200; break;
            case 2400: speed = CBR_2400; break;
            case 4800: speed = CBR_4800; break;
            case 9600: speed = CBR_9600; break;
            case 19200: speed = CBR_19200; break;
            case 38400: speed = CBR_38400; break;
            case 57600: speed = CBR_57600; break;
            case 115200: speed = CBR_115200; break;
            case 128000: speed = CBR_128000; break;
            case 256000: speed = CBR_256000; break;
            default:
                throw std::runtime_error("Invalid baud rate");
        }

        serialParameters.BaudRate = speed;
        serialParameters.ByteSize = 8;
        serialParameters.StopBits = ONESTOPBIT;
        serialParameters.fParity = NOPARITY;
        serialParameters.fInX = xOnOff ? 1 : 0;
        serialParameters.fOutX = xOnOff ? 1 : 0;
        serialParameters.fRtsControl = rtsCts ? 1 : 0;

        if (!SetCommState(handle, &serialParameters))
            throw std::runtime_error("Failed to set parameters");

#else
        fd = open(path.c_str(), O_RDWR | O_NOCTTY);

        if (fd == -1)
            throw std::runtime_error("Failed to open " + path);

        struct termios settings;
        if (tcgetattr(fd, &settings) != 0)
            throw std::runtime_error("Failed to get attributes");

        speed_t speed;

        switch (baud)
        {
            case 0: speed = B0; break;
            case 50: speed = B50; break;
            case 75: speed = B75; break;
            case 110: speed = B110; break;
            case 134: speed = B134; break;
            case 150: speed = B150; break;
            case 200: speed = B200; break;
            case 300: speed = B300; break;
            case 600: speed = B600; break;
            case 1200: speed = B1200; break;
            case 1800: speed = B1800; break;
            case 2400: speed = B2400; break;
            case 4800: speed = B4800; break;
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            default:
                throw std::runtime_error("Invalid baud rate");
        }

        if (cfsetospeed(&settings, speed) != 0) // baud rate
            throw std::runtime_error("Failed to set baud rate");
        if (cfsetispeed(&settings, speed) !=0)
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
#endif
    }

    ~Device()
    {
#ifdef _WIN32
        if (handle == INVALID_HANDLE_VALUE) CloseHandle(handle);
#else
        if (fd != -1) close(fd);
#endif
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

#ifdef _WIN32
        DWORD remaining = static_cast<DWORD>(data.size());
        const uint8_t* ptr = static_cast<const uint8_t*>(data.data());

        DWORD bytesWritten;
        if (!WriteFile(handle, ptr, remaining, &bytesWritten, nullptr))
            throw std::runtime_error("Failed to read data");

        ptr += bytesWritten;
        remaining -= bytesWritten;
#else
        int remaining = static_cast<int>(data.size());
        const uint8_t* ptr = static_cast<const uint8_t*>(data.data());

        while (remaining)
        {
            ssize_t result = write(fd, ptr, remaining);

            if (result == -1) throw std::runtime_error("Failed to write data");

            ptr += result;
            remaining -= result;
        }
#endif
    }

    std::vector<uint8_t> readData()
    {
        uint8_t buffer[256];
        std::vector<uint8_t> data;

#ifdef _WIN32
        DWORD bytesRead;
        if (!ReadFile(handle, buffer, sizeof(buffer), &bytesRead, nullptr))
            throw std::runtime_error("Failed to read data");

        data.assign(buffer, buffer + bytesRead);
#else
        ssize_t result = read(fd, buffer, sizeof(buffer));

        if (result == -1) throw std::runtime_error("Failed to read data");
        else if (result == 0)
        {
            close(fd);
            fd = -1;
            throw std::runtime_error("Connection closed");
        }

        data.assign(buffer, buffer + result);
#endif

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
        uint32_t baud = CEL_BAUD;
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
