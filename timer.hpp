#ifndef TIMER_HPP
#define TIMER_HPP

#include <time.h>

class Timer {
public:
    void Start()
    {
        begin = clock();
    }
    
    void Stop()
    {
        end = clock();
    }
    
    double Duration()
    {
        return double (end - begin) / CLOCKS_PER_SEC;
    }
    
private:
    clock_t begin;
    clock_t end;
};

#endif
