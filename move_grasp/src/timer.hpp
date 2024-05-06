#include <chrono>

class Timer
{
public:
    Timer()
    {
        reset();
    }

    void reset()
    {
        start = std::chrono::high_resolution_clock::now();
    }

    double elapsed()
    {
        return std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
    }

    double elapsed_ms()
    {
        return elapsed() * 1000;
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
};
