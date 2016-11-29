// PowerControl.cpp : main project file.

#include "stdafx.h"

using namespace System;

#include <vector>
#include <algorithm>
#include <functional>

#include <stdio.h>
#include <stddef.h>
#include <math.h>

/*! \brief number of binary digits after comma used in fixed point calculations */
#define PID_FIXED_POINT_POSITION (1 << 8)

#define KP (500)   // W/(0.1°C)
#define KPDIV (10) // compensate 0.1°C
#define KI (5)
#define MAXOUT 5000
#define MINOUT 2000
#define TARGET_X 80

#define ANTI_WINDUP_METHOD (1)

class PIControl
{
    int kp;
    int ki;
    int maxY;
    int minY;
    int prevPartI;
    int prevDeltaX;

  public:
    PIControl();  // constructor
    ~PIControl(); // de-constructor
    int control(int, int);
    void reset(void) { prevPartI = 0; };
    void with_kp(int x) { kp = (x * PID_FIXED_POINT_POSITION) / KPDIV; };
    void with_ki(int x) { ki = (x * PID_FIXED_POINT_POSITION) / KPDIV; };
    void with_maxY(int x) { maxY = x * PID_FIXED_POINT_POSITION; };
    void with_minY(int x) { minY = x * PID_FIXED_POINT_POSITION; };
};

PIControl::PIControl()
{
    kp         = 0;
    ki         = 0;
    maxY       = 0;
    minY       = 0;
    prevPartI  = 0;
    prevDeltaX = 0;
}

PIControl::~PIControl()
{
    printf("delete\n");
}

int PIControl::control(int xTarget, int xActual)
{

    Int32 deltaX; /* value of actual control deviation 'w' */
    Int32 partP;  /* value of the proportional part 'i' of the calculated setpoint */
    Int32 partI;  /* value of the integral part 'i' of the calculated setpoint */
    Int32 y;      /* calculated value of the pid controller in fixed-point representation */
    Int32 yLimited;
    static Int32 yCutoff = 0;

    /* Calculate control deviation */
    deltaX = xTarget - xActual;
    printf(" dx=%d ", deltaX);

    /* Calculate P- and D-part of setpoint */
    partP = deltaX * kp;

    /* Calculate I-part. */
    partI = (deltaX * ki) + prevPartI;

    if (partI > maxY)
    {
        partI = maxY;
    }
    else if (partI < minY)
    {
        partI = minY;
    }

    /* Add P- and I-part to the set-value */
    y = partP + partI;

    /* Check range and restrict the set-value if necessary */
    if (y > maxY)
    {
        yLimited = maxY;
        printf(" max ");
    }
    else if (y < minY)
    {
        yLimited = minY;
        printf(" min ");
    }
    else
    {
#if (ANTI_WINDUP_METHOD == 1)
        prevPartI = partI;
#endif
        yLimited = y;
        printf("     ");
    }

    yCutoff = y - yLimited;

#if (ANTI_WINDUP_METHOD == 2)
    prevPartI = partI - yCutoff;
#endif

    prevDeltaX = deltaX;

    printf(" yCutoff=%d ", yCutoff / 256);
    printf(" p=%d i=%d ", partP / 256, prevPartI / 256);

    return (Int16)((yLimited + (PID_FIXED_POINT_POSITION / 2)) / PID_FIXED_POINT_POSITION);
}

class SRControl
{
    signed int py;
    signed int maxrsr;
    signed int maxfsr;
    signed int scaleFactor;

  public:
    SRControl(int f, int rsra, int rsrb, int fsra, int fsrb)
    {
        scaleFactor = f;
        maxrsr      = (rsra * scaleFactor) / rsrb;
        maxfsr      = (fsra * scaleFactor) / fsrb;
        printf(" maxrsr = %d ", maxrsr);
        printf(" maxfsr = %d ", maxfsr);
    };

    void preset(int a, int b)
    {
        py = (a * scaleFactor) / b;
        printf("preset=%d\n", py / scaleFactor);
    };

    int limitSlewRate(int x)
    {
        int dx = x * scaleFactor - py;
        if (dx > maxrsr)
            dx = maxrsr;
        else if (dx < maxfsr)
            dx = maxfsr;
        py     = py + dx;
        return py / scaleFactor;
    }
};

//int main(array<System::String ^> ^args)
//{
//    Console::WriteLine(L"Hello World");
//    return 0;
//}

int main(int argc, const char *argv[])
{
    int startY, actualX, y, y1, py;
    int guidedTargetTemperature;
    int guidedActualTemperature;
    double x, time;

    PIControl mainc;

    SRControl powerSRC(1, +83, 1, -83, 1);

    SRControl targetSRC(100, +830, (KP + KI), -830, (KP + KI));

    SRControl measuredSRC(100, +830, (KP + KI), -830, (KP + KI));

    if (argv[1])
    {
        startY  = atoi(argv[1]);
        actualX = atoi(argv[2]);
    }
    else
    {
        printf("error: number needed.\n");
        return -1;
    }

    powerSRC.preset(startY,
                    1);

    targetSRC.preset(10 * ((actualX * (KP + KI)) + (startY - MINOUT)),
                     (KP + KI));

    measuredSRC.preset(10 * actualX,
                       1);

    mainc.with_kp(KP);
    mainc.with_ki(KI);
    mainc.with_maxY(MAXOUT);
    mainc.with_minY(MINOUT);

    for (time = 0; time < 400; time += 1)
    {
        if (time < 50)
            x = actualX;
        else
            x = TARGET_X;

        printf("time=%2.2f temperature=%.1f", time, x);

        guidedTargetTemperature = targetSRC.limitSlewRate(10 * TARGET_X);

        guidedActualTemperature = measuredSRC.limitSlewRate(10 * (signed int)x);

        y = mainc.control(guidedTargetTemperature,
                          guidedActualTemperature);

        y1 = powerSRC.limitSlewRate(y);

        printf("\t s=%d ", guidedTargetTemperature);
        printf("\t m=%d ", guidedActualTemperature);
        printf("\t y=%d ", y);
        printf("\t dy=%d ", y1 - py);
        printf("\n");
        py = y1;
    }
}
