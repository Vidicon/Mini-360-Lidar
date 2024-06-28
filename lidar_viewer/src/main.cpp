#include <thread>
#include <atomic>
#include  <iostream>
#include  <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <vector>
#include <termios.h>
#include <cstring>
#include <cstdint>
#include <SDL2/SDL.h> // sudo apt install libsdl2-dev

#include "messageQueue.hpp"
// #if __has_include(<format>)
//     #include <format>
//     using std::format;
// #else
//     #include <fmt/format.h>
//     using fmt::format;
// #endif

// Screen dimension constants
int SCREEN_WIDTH = 800;
int SCREEN_HEIGHT = 600;

std::atomic<bool> running(true);
const uint16_t BUFFER_SIZE = 1024; 
uint8_t circle_buffer[BUFFER_SIZE];
uint16_t head = 0;
uint16_t tail = 0;

struct LidarData
{
   double start_angle;
   double step_size;
   double distances[16];
   double intensities[16];
   uint8_t xdata[16];
};

std::vector<LidarData> lidarDataBuffer;
double last_angle = 0.0;

struct Color
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};


uint8_t header[] = {0x55, 0xAA};

Color value_to_rgb(int value) {
    if (value < 0 || value > 255) {
        throw std::invalid_argument("Value must be between 0 and 255");
    }

    int r, g, b;

    if (value >= 0 && value <= 42) {
        // Black to Blue
        r = static_cast<int>(((42 - value) / 42.0) * 255);
        g = static_cast<int>(((42 - value) / 42.0) * 255);
        b = 255;
    } else if (value >= 43 && value <= 85) {
        // Blue to Cyan
        r = 0;
        g = static_cast<int>(((value - 43) / 42.0) * 255);
        b = 255;
    } else if (value >= 86 && value <= 127) {
        // Cyan to Green
        r = 0;
        g = 255;
        b = 255 - static_cast<int>(((value - 86) / 42.0) * 255);
    } else if (value >= 128 && value <= 170) {
        // Green to Yellow
        r = static_cast<int>(((value - 128) / 42.0) * 255);
        g = 255;
        b = 0;
    } else if (value >= 171 && value <= 212) {
        // Yellow to Orange
        r = 255;
        g = 255 - static_cast<int>(((value - 171) / 42.0) * 255);
        b = 0;
    } else if (value >= 213 && value <= 255) {
        // Orange to Red
        r = 255;
        g = static_cast<int>(((value - 213) / 42.0) * 165);
        b = 0;
    }

    return Color{ static_cast<Uint8>(r), static_cast<Uint8>(g), static_cast<Uint8>(b) };
}

// Function to convert polar coordinates to Cartesian
void polarToCartesian(float r, float theta, float& x, float& y) {
    x = r * cos(theta);
    y = r * sin(theta);
}

void drawPolarPlot(SDL_Renderer* renderer, float zoom, MessageQueue<LidarData>& mq) {
    SDL_GetRendererOutputSize(renderer, &SCREEN_WIDTH, &SCREEN_HEIGHT);
    int centerX = SCREEN_WIDTH / 2;
    int centerY = SCREEN_HEIGHT / 2;

    float r, theta;
    float x, y;



    
    
    while(!mq.empty())
    {
        LidarData lidarData = mq.pop();
        lidarDataBuffer.push_back(lidarData);
        if(lidarData.start_angle< last_angle)
        {
            SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255); // White color
            SDL_RenderClear(renderer);
            SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255); // White color
            SDL_Rect rect;
            rect.x = centerX - 6; // Adjust to center the square
            rect.y = centerY - 6; // Adjust to center the square
            rect.w = 12;
            rect.h = 12;
            SDL_RenderFillRect(renderer, &rect);

            for(LidarData lD : lidarDataBuffer)
            {
                for (int i = 0; i < 16; i++)
                {
                    double theta = (lD.start_angle + lD.step_size * i) * M_PI / 180.0;
                    polarToCartesian(lD.distances[i] * zoom, -theta, x, y);
                    // Color c = value_to_rgb(lD.intensities[i]);
                    Color c = value_to_rgb(lD.xdata[i] * 32 + 80);
                    
                    SDL_SetRenderDrawColor(renderer, c.r, c.g, c.b, 255); // Blue color

                    SDL_Rect rect;
                    rect.x = centerX + static_cast<int>(x) - 1; // Adjust to center the square
                    rect.y = centerY - static_cast<int>(y) - 1; // Adjust to center the square
                    rect.w = 3;
                    rect.h = 3;
                    SDL_RenderFillRect(renderer, &rect);
                }
            }

            SDL_RenderPresent(renderer);
            lidarDataBuffer.clear();
        }

        last_angle = lidarData.start_angle;
      
    }

    
}

void guiThread(MessageQueue<LidarData>& mq)
{
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL could not initialize! SDL_Error: " << SDL_GetError() << std::endl;
        return;
    }

    // Create window
    SDL_Window* window = SDL_CreateWindow("Polar Plot", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_RESIZABLE);
    if (window == NULL) {
        std::cerr << "Window could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return;
    }

    // Create renderer for window
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (renderer == NULL) {
        std::cerr << "Renderer could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return;
    }

    bool quit = false;
    SDL_Event e;
    float zoom = 1.0f;
    const int FRAME_RATE = 60;
    const int FRAME_DELAY = 1000 / FRAME_RATE;

    // Main loop
    while (!quit) {

        Uint32 frameStart = SDL_GetTicks();
        // Handle events on the queue
        while (SDL_PollEvent(&e) != 0) {
            // User requests quit
            if (e.type == SDL_QUIT) {
                quit = true;
            } else if (e.type == SDL_KEYDOWN) {
                // Handle key presses
                switch (e.key.keysym.sym) {
                    case SDLK_KP_PLUS:
                        zoom *= 1.1f;
                        break;
                    case SDLK_KP_MINUS:
                        zoom /= 1.1f;
                        break;
                }
            } else if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_RESIZED) {
                // Handle window resize
                int width, height;
                SDL_GetWindowSize(window, &width, &height);
                SDL_RenderSetLogicalSize(renderer, width, height);
            }
        }

        // Draw the polar plot
        drawPolarPlot(renderer, zoom, std::ref(mq));

        Uint32 frameTime = SDL_GetTicks() - frameStart;
        if (frameTime < FRAME_DELAY) {
            SDL_Delay(FRAME_DELAY - frameTime);
        }
    }

    // Clean up and close SDL
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

int configureSerialPort(const char* portName, int baudRate) {
    int fd = open(portName, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening serial port: " << strerror(errno) << std::endl;
        return -1;
    }

    termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, baudRate);
    cfsetispeed(&tty, baudRate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag |= 0;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    return fd;
}

void addToBuffer(uint8_t data) {
    circle_buffer[head] = data;
    head = (head + 1) % BUFFER_SIZE;
    if (head == tail) {
        tail = (tail + 1) % BUFFER_SIZE;
        std::cerr << "Buffer overflow" << std::endl;
    }
}

uint16_t available() {
    return (BUFFER_SIZE + head - tail) % BUFFER_SIZE;
}

uint16_t freeSpace() {
    return (BUFFER_SIZE - 1 - head + tail) % BUFFER_SIZE;
}

uint16_t removeBuffer(uint16_t size) {
    uint16_t pop_count = 0;
    for (uint16_t i = 0; i < size; i++) {
        if (available() > 0) { // Check if data is available in the buffer
            tail = (tail + 1) % BUFFER_SIZE;
            pop_count++;
        }
    }
    return pop_count;
}

uint16_t peakBuffer(uint8_t* data, uint16_t size) {
    uint16_t peak_count = 0;
    for (uint16_t i = 0; i < size; i++) {
        if (available() > 0) { // Check if data is available in the buffer
            data[i] = circle_buffer[tail+i];
            peak_count++;
        }
    }
    return peak_count;
}
uint16_t readFromBuffer(uint8_t* data, uint16_t size) {
    uint16_t read_count = 0;
    for (uint16_t i = 0; i < size; i++) {
        if (available() > 0) { // Check if data is available in the buffer
            data[i] = circle_buffer[tail];
            tail = (tail + 1) % BUFFER_SIZE;
            read_count++;
        }
    }
    return read_count;
}

void readSerialPort(int fd) {
    uint8_t buf[1];
    while (running) {
        int n = read(fd, buf, sizeof(buf));
        if (n > 0) {
            // std::cout << std::hex << (int) buf[0] << std::endl;
            addToBuffer(buf[0]);
        } else if (n < 0) {
            std::cerr << "Error reading: " << strerror(errno) << std::endl;
            running = false;
        }
    }
}

bool FindHeader()
{
    while(true)
    {
        if(available() < 2)
        {
            return false;
        }
        else
        {
            uint8_t data[2];
            uint16_t size = peakBuffer(data, 2);
            if (size == 2) {
                // std::cout << "Data: " << std::hex << (int) data[0] << " " << (int) data[1] << std::endl;
                if (data[0] == header[0] && data[1] == header[1]) {
                    return true;
                }
                else
                {
                    removeBuffer(1);
                }
            }
        }
    }
    
}

void printData(uint8_t* data, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++) {
        std::cout << std::setfill('0') << std::setw(2) << std::hex << (int) data[i] << " ";
    }
}

void printBin(uint8_t data)
{
    for (uint16_t i = 0; i < 8; i++) {
        std::cout << (int) ((data >> (7-i)) & 1);
    }
}

LidarData parseData(uint8_t* data, uint16_t size)
{
    uint8_t type = data[2];
    uint8_t data_size = data[3];

    LidarData lidarData;
    if(type == 0x23)
    {
        uint16_t speed = (data[5] << 8) | data[4];
        uint16_t start_angle = (((data[7] & 0x7F) << 8) + data[6]) - 0x2000;
        uint16_t end_angle =   (((data[57] & 0x7F) << 8) | data[56]) - 0x2000;
        uint16_t crc = (data[59] << 8) | data[58];
        double start_angle_deg = start_angle / 64.0;
        double end_angle_deg = end_angle / 64.0;
        
        double dif = end_angle_deg - start_angle_deg;
        if(end_angle_deg < start_angle_deg)
        {
            dif = 360.0 - start_angle_deg + end_angle_deg;
        }

        lidarData.start_angle = start_angle_deg;
        lidarData.step_size = dif / (data_size - 1);

        for (int i = 0; i < data_size; i++)
        {
            uint16_t offset = i * 3;
            uint16_t distance = (((data[9+offset] & 0x3F) << 8) | data[8+offset]) * 0.1;
            uint8_t xdata = data[9+offset] >> 6;
            uint8_t strength = data[10+offset];


            double sample_angle = start_angle_deg + (dif / (data_size - 1)) * i;
            if(sample_angle >= 360.0)
            {
                sample_angle -= 360.0;
            }

            lidarData.distances[i] = distance;
            if(strength == 0){
                lidarData.distances[i] = 0;
            }
            lidarData.xdata[i] = xdata;
            lidarData.intensities[i] = strength;
            // if(sample_angle < 0.8   )
            // {
            //     std::cout << "Angle: " << std::fixed << std::setprecision(2) << sample_angle << \
            //     " \tDistance: " << std::dec << std::setfill(' ') << std::setw(8) << distance << \
            //     " \tdata: " << std::hex << std::setfill('0') << std::setw(2) << std::right << (int) data[8+offset] << " " << \
            //     std::setw(2) << std::right << (int) data[9+offset] << " " << std::setw(2) << std::right << (int) data[10+offset];
            //     // " \tQ: " << q <<std::endl;
            //     std::cout << " \t";
            //     printBin(data[8+offset]);
            //     std::cout << " \t";
            //     printBin(data[9+offset]);
            //     std::cout << " \t";
            //     printBin(data[10+offset]);
            //     std::cout << std::endl;
            // }
        }
        

        
    }
    return lidarData;
}

int main() {
    const char* portName = "/dev/ttyUSB0";  // Change this to your serial port
    int baudRate = 230400;  // Change this to your baud rate

    int serialPort = configureSerialPort(portName, baudRate);
    if (serialPort < 0) {
        return 1;
    }

    MessageQueue<LidarData> mq;

    std::thread reader(readSerialPort, serialPort);
    std::thread gui(guiThread, std::ref(mq));

    while (true) {
        uint16_t size = available();
        // std::cout << "Available data: " << std::dec << size << std::endl;


        
        if(size > 2)
        {
            if(FindHeader())
            {
                // std::cout << "Found Header" << std::endl;
                while(available() < 60)
                {
                    // std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                uint8_t data[60];
                uint16_t size = readFromBuffer(data, 60);
                LidarData lidarData = parseData(data, size);
                mq.push(lidarData);
            }
        }
        

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    reader.join();
    gui.join();
    close(serialPort);

    return 0;
}
