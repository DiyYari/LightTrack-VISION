/*
 *	An Arduino library for the Hi-Link LD2450 24Ghz FMCW radar sensor.
 *
 *  This sensor is a Frequency Modulated Continuous Wave radar, which makes it good for presence detection and its sensitivity at different ranges to both static and moving targets can be configured.
 *
 *	The code in this library is based off the https://github.com/0ingchun/arduino-lib_HLK-LD2450_Radar.
 *
 *	https://github.com/ncmreynolds/ld2410
 *
 *
 */
#ifndef LD2450_h
#define LD2450_h




#include <Arduino.h>

#ifdef SoftwareSerial_h
    #define ENABLE_SOFTWARESERIAL_SUPPORT
#endif


#ifdef ENABLE_SOFTWARESERIAL_SUPPORT
    #include <SoftwareSerial.h>
#endif

#define LD2450_MAX_SENSOR_TARGETS 3
#define LD2450_SERIAL_BUFFER 256
#define LD2450_SERIAL_SPEED 256000
#define LD2450_DEFAULT_RETRY_COUNT_FOR_WAIT_FOR_MSG 1000





class LD2450
{

public:
    typedef struct RadarTarget
    {
        uint16_t id;         // ID
        int16_t x;           // X mm
        int16_t y;           // Y mm
        int16_t speed;       // cm/s
        uint16_t resolution; // mm
        uint16_t distance; // mm calculated from  x y 
        bool valid;
    } RadarTarget_t;

    LD2450();
    // Constructor function
    ~LD2450();

    void begin(Stream &radarStream);
    void begin(HardwareSerial &radarStream, bool already_initialized = false);

#ifdef ENABLE_SOFTWARESERIAL_SUPPORT
    void begin(SoftwareSerial &radarStream, bool already_initialized = false);
#endif

    bool waitForSensorMessage(bool wait_forever = false);
    void setNumberOfTargets(uint16_t _numTargets);
    int ProcessSerialDataIntoRadarData(byte rec_buf[], int len);
    RadarTarget getTarget(uint16_t _target_id);
    uint16_t getSensorSupportedTargetCount();
    String getLastTargetMessage();
    int read();

protected:

private:
    Stream *radar_uart = nullptr;
    RadarTarget_t radarTargets[LD2450_MAX_SENSOR_TARGETS]; // Stores the target of the current frame
    uint16_t numTargets = LD2450_MAX_SENSOR_TARGETS;
    String last_target_data = "";
};
#endif
