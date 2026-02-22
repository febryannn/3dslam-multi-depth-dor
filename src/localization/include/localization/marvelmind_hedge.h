#ifndef MARVELMIND_HEDGE_H_
#define MARVELMIND_HEDGE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef WIN32
    #define WIN32_LEAN_AND_MEAN
    #define NOMINMAX
    #include <Windows.h>
#else
    // Wajib ada untuk tipe data pthread_t dan pthread_mutex_t di Linux
    #include <pthread.h>
    #include <semaphore.h>
#endif

#define DATA_INPUT_SEMAPHORE "/data_input_semaphore"

typedef union {
    uint32_t timestamp32;
    int64_t timestamp64;
} TimestampOpt;

struct PositionValue
{
    uint8_t address;
    TimestampOpt timestamp;
    bool realTime;
    int32_t x, y, z; // koordinat dalam mm
    uint8_t flags;
    double angle;
    bool highResolution;
    bool ready;
    bool processed;
};

struct RawIMUValue
{
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t compass_x, compass_y, compass_z;
    TimestampOpt timestamp;
    bool realTime;
    bool updated;
};

struct FusionIMUValue
{
    int32_t x, y, z; // mm
    int16_t qw, qx, qy, qz; // quaternion (normalized to 10000)
    int16_t vx, vy, vz; // mm/s
    int16_t ax, ay, az; // mm/s^2
    TimestampOpt timestamp;
    bool realTime;
    bool updated;
};

struct RawDistanceItem
{
    uint8_t address_beacon;
    uint32_t distance; // mm
};

struct RawDistances
{
    uint8_t address_hedge;
    struct RawDistanceItem distances[4];
    TimestampOpt timestamp;
    bool realTime;
    uint16_t timeShift;
    bool updated;
};

struct StationaryBeaconPosition
{
    uint8_t address;
    int32_t x, y, z; // mm
    bool updatedForMsg;
    bool highResolution;
};

#define MAX_STATIONARY_BEACONS 255
struct StationaryBeaconsPositions
{
    uint8_t numBeacons;
    struct StationaryBeaconPosition beacons[MAX_STATIONARY_BEACONS];
    bool updated;
};

struct TelemetryData
{
    uint16_t vbat_mv;
    int8_t rssi_dbm;
    bool updated;
};

struct QualityData
{
    uint8_t address;
    uint8_t quality_per;
    bool updated;
};

#define MAX_WAYPOINTS_NUM 255
struct WaypointData
{
    uint8_t movementType;
    int16_t param1, param2, param3;
    bool updated;
};

struct WaypointsData
{
    uint8_t numItems;
    struct WaypointData items[MAX_WAYPOINTS_NUM];
    bool updated;
};

struct UserPayloadData
{
    TimestampOpt timestamp;
    uint8_t data[256];
    uint8_t dataSize;
    bool updated;
};

struct MarvelmindHedge
{
    const char * ttyFileName;
    uint32_t baudRate;
    uint8_t maxBufferedPositions;
    struct PositionValue * positionBuffer;
    struct StationaryBeaconsPositions positionsBeacons;
    struct RawIMUValue rawIMU;
    struct FusionIMUValue fusionIMU;
    struct RawDistances rawDistances;
    struct TelemetryData telemetry;
    struct QualityData quality;
    struct WaypointsData waypoints;
    struct UserPayloadData userPayloadData;

    bool verbose;
    bool pause;
    bool terminationRequired;

    void (*receiveDataCallback)(struct PositionValue position);
    void (*anyInputPacketCallback)();

    uint8_t lastValuesCount_;
    uint8_t lastValues_next;
    bool haveNewValues_;

#ifdef WIN32
    CRITICAL_SECTION lock_;
#else
    // Menggunakan standar POSIX untuk Linux
    pthread_t thread_;
    pthread_mutex_t lock_;
#endif
};

// Datagram IDs
#define POSITION_DATAGRAM_ID 0x0001
#define BEACONS_POSITIONS_DATAGRAM_ID 0x0002
#define POSITION_DATAGRAM_HIGHRES_ID 0x0011
#define BEACONS_POSITIONS_DATAGRAM_HIGHRES_ID 0x0012
#define IMU_RAW_DATAGRAM_ID 0x0003
#define BEACON_RAW_DISTANCE_DATAGRAM_ID 0x0004
#define IMU_FUSION_DATAGRAM_ID 0x0005
#define TELEMETRY_DATAGRAM_ID 0x0006
#define QUALITY_DATAGRAM_ID 0x0007
#define NT_POSITION_DATAGRAM_HIGHRES_ID 0x0081
#define NT_IMU_RAW_DATAGRAM_ID 0x0083
#define NT_BEACON_RAW_DISTANCE_DATAGRAM_ID 0x0084
#define NT_IMU_FUSION_DATAGRAM_ID 0x0085
#define WAYPOINT_DATAGRAM_ID 0x0201
#define GENERIC_USER_DATA_DATAGRAM_ID 0x0280

// Prototipe Fungsi
struct MarvelmindHedge * createMarvelmindHedge ();
void destroyMarvelmindHedge (struct MarvelmindHedge * hedge);
void startMarvelmindHedge (struct MarvelmindHedge * hedge);
void stopMarvelmindHedge (struct MarvelmindHedge * hedge);

bool getPositionFromMarvelmindHedge (struct MarvelmindHedge * hedge, struct PositionValue * position);
bool getStationaryBeaconsPositionsFromMarvelmindHedge (struct MarvelmindHedge * hedge, struct StationaryBeaconsPositions * positions);
void clearStationaryBeaconUpdatedFlag(struct MarvelmindHedge * hedge, uint8_t address);

#ifdef WIN32
    #define DEFAULT_TTY_FILENAME "\\\\.\\COM3"
#else
    #define DEFAULT_TTY_FILENAME "/dev/ttyACM0"
#endif

#define DEFAULT_TTY_BAUDRATE 9600UL

#endif