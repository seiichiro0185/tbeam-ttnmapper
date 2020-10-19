#ifndef __GPS_H__
#define __GPS_H__

#include <TinyGPS++.h>
#include <HardwareSerial.h>

class Gps
{
    public:
        void init();
        bool checkGpsFix();
        bool buildPacket(uint8_t txBuffer[9]);
        void gdisplay(uint16_t txBuffer2[5]);
        void encode();

    private:
        uint32_t LatitudeBinary, LongitudeBinary;
        uint16_t altitudeGps;
        uint8_t hdopGps;
        char t[32]; // used to sprintf for Serial output
        TinyGPSPlus tGps;
        TinyGPSCustom satsInView; // Satellites in View
};

#endif
