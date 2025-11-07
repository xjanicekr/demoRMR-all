////*************************************************************************************
////*************************************************************************************
//// autor Martin Dekan  mail: dekdekan@gmail.com
////-------------------------------------------------------------------------------------
//// co to je:
//// aktualne nic, class je vycistena od veci ktore su na udp komunikacii zbytocne
/// na robote je rovnaka classa ktora riesi komunikaciu s lidarom.
/// Viac menej, je to tu len kvoli strukture, zmysel classy nehladajte
////*************************************************************************************
////*************************************************************************************
#pragma once
#ifdef _WIN32
#define NOMINMAX
#include<windows.h>
#else
#include<arpa/inet.h>
#include<sys/socket.h>

#endif
//#include <winsock2.h>
#include <stdio.h>
#include <stdlib.h>
//#include "unistd.h"
//#include "thread.h"
#include "iostream"
#include "fcntl.h"
#include "string.h"
#include <errno.h>
//#include <termios.h>
//#include <unistd.h>
#include <stdio.h>

#include<iostream>

//#include<unistd.h>
//
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>



typedef struct
{
    int scanQuality;
    float scanAngle;
    float scanDistance;
    uint32_t timestamp;
}LaserData;

typedef struct
{
    int numberOfScans;
    LaserData Data[1000];
}LaserMeasurement;
class rplidar
{
public:
    int i;
    rplidar()
    {

    }

    virtual  ~rplidar()
    {

    }



};
