#ifndef LIBHINJ_H
#define LIBHINJ_H

#include <stdint.h>

/* error codes */
#define HINJ_NO_PATH    -1
#define HINJ_NO_SOCK    -2
#define HINJ_NO_CONN    -3
#define HINJ_BAD_RECV   -4
#define HINJ_BAD_SEND   -5
#define HINJ_BAD_BIND   -6
#define HINJ_BAD_LISTEN -7

#define HINJ_MODE_ENCODE(bmode, cmode, smode) \
        (((bmode) << 16) | ((cmode) << 8) | (smode))

/* message types */
const uint8_t HINJ_HIL_GPS              = 0;
const uint8_t HINJ_HIL_SENSOR           = 1;
const uint8_t HINJ_HIL_RC_INPUTS_RAW    = 2;
const uint8_t HINJ_HIL_STATE_QUATERNION = 3;
const uint8_t HINJ_HIL_ACCEL            = 4;
const uint8_t HINJ_HIL_GYRO             = 5;
const uint8_t HINJ_HIL_BATTERY          = 6;
const uint8_t HINJ_HIL_COMPASS          = 7;
const uint8_t HINJ_HIL_BAROMETER        = 8;
const uint8_t HINJ_HIL_MODE             = 9;

/* action types */
#define HINJ_IGNORE_SENSOR 1

struct compass_pkt {
        uint8_t  message_type;
        uint32_t message_size;
        uint8_t  instance;
        uint8_t  ignore;
        float    mag_0;
        float    mag_1;
        float    mag_2;
} __attribute__((packed));

struct barometer_pkt {
        uint8_t  message_type;
        uint32_t message_size;
        uint8_t  instance;
        uint8_t  ignore;
        float    pressure;
        float    temperature;
} __attribute__((packed));

struct battery_pkt {
        uint8_t  message_type;
        uint32_t message_size;
        float    voltage;
        float    current;
        float    throttle;
} __attribute__((packed));

struct gps_pkt {
        uint8_t  message_type;
        uint32_t message_size;
        uint8_t  instance;
        uint8_t  ignore;
        uint64_t time_usec;
        uint8_t  fix_type;
        int32_t  lat;
        int32_t  lon;
        int32_t  alt;
        uint16_t eph;
        uint16_t epv;
        uint16_t vel;
        int16_t  vn;
        int16_t  ve;
        int16_t  vd;
        uint16_t cog;
        uint8_t  satellites_visible;
} __attribute__((packed));

struct accel_pkt {
        uint8_t  message_type;
        uint32_t message_size;
        uint8_t  instance;
        uint8_t  ignore;
        float    accelx;
        float    accely;
        float    accelz;
} __attribute__((packed));

struct gyro_pkt {
        uint8_t  message_type;
        uint32_t message_size;
        uint8_t  instance;
        uint8_t  ignore;
        float    x;
        float    y;
        float    z;
} __attribute__((packed));

struct mode_pkt {
        uint8_t  message_type;
        uint32_t message_size;
        uint32_t mode;
} __attribute__((packed));

/* 
 * synchronizes the activities of fdm, autopilot, and framework
 *
 *  - returns a non-zero error-code on error
 */
int hinj_start_sync();

/* 
 * synchronizes the activities of fdm, autopilot, and framework
 * hinj_start_sync() must be called first
 *
 *  - returns a non-zero error-code on error
 */
int hinj_end_sync();

/* 
 * updates the gyro values
 *  - param0: x
 *  - param1: y
 *  - param2: z
 *  - returns a non-zero error-code on error
 */
int update_gyro(float *, float *, float *, uint8_t);

/* 
 * updates the accel values
 *  - param0: accel x
 *  - param1: accel y
 *  - param2: accel z
 *  - returns a non-zero error-code on error
 */
int update_accel(float *, float *, float *, uint8_t);

/* 
 * updates the GPS values
 *
 *  - returns a non-zero error-code on error
 */
int update_gps(struct gps_pkt *);

/* 
 * updates the values of the compass
 *   - param0: mag0
 *   - param1: mag1
 *   - param2: mag2
 *
 *   - returns a non-zero error-code on error and leaves the original compass unmodified
 */
int update_compass(float *, float *, float *, uint8_t);

/* 
 * updates the values of pressure and temperature respectively
 *  - param0: temperature
 *  - param1: pressure
 *
 *  - returns a non-zero error-code on error and leaves the original barometer unmodified
 */
int update_barometer(float *, float *, uint8_t);

/* 
 * updates the values of voltage, current, and throttle
 *   - param0: voltage
 *   - param1: current
 *   - param2: throttle
 *
 *   - returns a non-zero error-code on error and leaves the original values unmodified
 */
int update_battery(float *, float *, float *);

/* 
 * returns a string representation of the provided error code
 *   - if the error code is not provided by this lib, returns an appropriate message
 */
const char* hinj_strerror(int);

/* 
 * sends the current mode
 */
int hinj_update_mode(int);

#endif
