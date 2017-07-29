//
//  timer.cpp
//  easyoperationofrobot
//
//  Created by venushka-mac on 15/4/15.
//  Copyright (c) 2015 uni.westminster. All rights reserved.
//


#include "timer.h"


#ifdef _WIN32
#include <windows.h>
#else
#include <mach/mach.h>
#include <sys/time.h>

#endif

Timer::Timer() : _clocks(0), _start(0)
{
#ifdef _WIN32
    QueryPerformanceFrequency((LARGE_INTEGER *)&_freq);
#else
    _freq = 1000;
#endif
}

Timer::~Timer()
{
}

void Timer::Start()
{
#ifdef _WIN32
    QueryPerformanceCounter((LARGE_INTEGER*)&_start);
#else
    struct timeval s;
    gettimeofday(&s, 0);
    _start = (i64)s.tv_sec * 1000 + (i64)s.tv_usec / 1000;
#endif
}

void Timer::Stop()
{
    i64 n;
    
#ifdef _WIN32
    QueryPerformanceCounter((LARGE_INTEGER*)&n);
#else
    struct timeval s;
    gettimeofday(&s, 0);
    n = (i64)s.tv_sec * 1000 + (i64)s.tv_usec / 1000;
#endif
    
    n -= _start;
    _start = 0;
    _clocks += n;
}

void Timer::Reset()
{
    _clocks = 0;
}

double Timer::GetElapsedTime()
{
    return (double)_clocks / (double)_freq;
}

