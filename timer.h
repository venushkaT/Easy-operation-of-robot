//
//  timer.h
//  easyoperationofrobot
//
//  Created by venushka-mac on 15/4/15.
//  Copyright (c) 2015 uni.westminster. All rights reserved.
//

#ifndef easyoperationofrobot_timer_h
#define easyoperationofrobot_timer_h


#ifdef _WIN32
typedef __int64 i64;
#else
typedef long long i64;
#endif

class Timer
{
public:
    Timer();
    ~Timer();
    void Start();
    void Stop();
    void Reset();
    double GetElapsedTime();
private:
    i64 _freq;
    i64 _clocks;
    i64 _start;
};


#endif
