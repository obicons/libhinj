#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include "libhinj.h"

/* GLOBALS */
static int sync_fd = -1;

/* INTERNAL FUNCTIONS - DO NOT USE OUTSIDE THIS PROJECT */

/* 
 * makes a unix socket at path
 *  - returns an error-code on error
 */
static int make_socket(const char *);

/* 
 * initializes the sync_fd 
 */
static void init_sync_fd();

/* 
 * returns a path suitable for sending messages
 *   - returns NULL on error
 */
static const char* make_msg_path();

/* 
 * returns the path to the synchronization file 
 *  - returns NULL on error
 */
static const char* make_sync_path();

int
hinj_start_sync()
{
        int f;
        if (sync_fd == -1)
                init_sync_fd();

        if (sync_fd < 0)
                return sync_fd;

        do {
                if ((f = accept(sync_fd, NULL, NULL)) != -1)
                        break;
                else
                        perror("accept() error");
        } while (1);

        close(f);

        return 0;
}

int
hinj_end_sync()
{
        int fd, e = 0;

        static long long iteration_count;
        if (sync_fd < 0)
                return sync_fd;

        do {
                if ((fd = accept(sync_fd, NULL, NULL)) != -1)
                        break;
                else
                        perror("accept() error");
        } while (1);

        if (write(fd, &iteration_count, sizeof(iteration_count)) != sizeof(iteration_count))
                e = HINJ_BAD_SEND;

        close(fd);
        iteration_count++;

        return e;
}

int
update_gyro(float *x, float *y, float *z, uint8_t instance)
{
        int fd, e = 0;
        const char *path;
        struct gyro_pkt cpkt = {
                .message_type = HINJ_HIL_GYRO,
                .message_size = sizeof(struct gyro_pkt),
                .instance = instance,
                .ignore = 0,
                .x = *x,
                .y = *y,
                .z = *z
        };

        if ((path = make_msg_path()) == NULL)
                return HINJ_NO_PATH;

        if ((fd = make_socket(path)) < 0)
                return fd;

        if (send(fd, &cpkt, sizeof(cpkt), 0) != sizeof(cpkt)) {
                e = HINJ_BAD_SEND;
                goto clean;
        }

        if (recv(fd, &cpkt, sizeof(cpkt), 0) != sizeof(cpkt)) {
                e = HINJ_BAD_RECV;
                goto clean;
        }

        if (cpkt.ignore)
                e = HINJ_IGNORE_SENSOR;
        
        *x = cpkt.x;
        *y = cpkt.y;
        *z = cpkt.z;

clean:
        close(fd);
        return e;
}

int
update_accel(float *x, float *y, float *z, uint8_t instance)
{
        int fd, e = 0;
        const char *path;
        struct accel_pkt cpkt = {
                .message_type = HINJ_HIL_ACCEL,
                .message_size = sizeof(struct accel_pkt),
                .instance = instance,
                .ignore = 0,
                .accelx = *x,
                .accely = *y,
                .accelz = *z
        };

        if ((path = make_msg_path()) == NULL)
                return HINJ_NO_PATH;

        if ((fd = make_socket(path)) < 0)
                return fd;

        if (send(fd, &cpkt, sizeof(cpkt), 0) != sizeof(cpkt)) {
                e = HINJ_BAD_SEND;
                goto clean;
        }

        if (recv(fd, &cpkt, sizeof(cpkt), 0) != sizeof(cpkt)) {
                e = HINJ_BAD_RECV;
                goto clean;
        }

        if (cpkt.ignore)
                e = HINJ_IGNORE_SENSOR;
        
        *x = cpkt.accelx;
        *y = cpkt.accely;
        *z = cpkt.accelz;

clean:
        close(fd);
        return e;
}

int
update_gps(struct gps_pkt *cpkt)
{
        int fd, e = 0;
        const char *path;

        if ((path = make_msg_path()) == NULL)
                return HINJ_NO_PATH;

        if ((fd = make_socket(path)) < 0)
                return fd;

        if (send(fd, cpkt, sizeof(struct gps_pkt), 0) != sizeof(struct gps_pkt)) {
                e = HINJ_BAD_SEND;
                goto clean;
        }

        if (recv(fd, cpkt, sizeof(struct gps_pkt), 0) != sizeof(struct gps_pkt)) {
                e = HINJ_BAD_RECV;
                goto clean;
        }

        if (cpkt->ignore)
                e = HINJ_IGNORE_SENSOR;

clean:
        close(fd);
        return e;        
}

int
update_compass(float *mag_0, float *mag_1, float *mag_2, uint8_t instance)
{
        int fd, e = 0;
        const char *path;
        struct compass_pkt cpkt = {
                .message_type = HINJ_HIL_COMPASS,
                .message_size = sizeof(struct compass_pkt),
                .instance = instance,
                .mag_0 = *mag_0,
                .mag_1 = *mag_1,
                .mag_2 = *mag_2
        };

        if ((path = make_msg_path()) == NULL)
                return HINJ_NO_PATH;

        if ((fd = make_socket(path)) < 0)
                return fd;

        if (send(fd, &cpkt, sizeof(cpkt), 0) != sizeof(cpkt)) {
                e = HINJ_BAD_SEND;
                goto clean;
        }

        if (recv(fd, &cpkt, sizeof(cpkt), 0) != sizeof(cpkt)) {
                e = HINJ_BAD_RECV;
                goto clean;
        }
        
        *mag_0 = cpkt.mag_0;
        *mag_1 = cpkt.mag_1;
        *mag_2 = cpkt.mag_2;

        if (cpkt.ignore)
                e = HINJ_IGNORE_SENSOR;
 
clean:
        close(fd);
        return e;
}

int
update_battery(float *voltage, float *current, float *throttle)
{
        int fd, e = 0;
        const char *path;
        struct battery_pkt cpkt = {
                .message_type = HINJ_HIL_BATTERY,
                .message_size = sizeof(struct battery_pkt),
                .voltage = *voltage,
                .current = *current,
                .throttle = *throttle
        };

        if ((path = make_msg_path()) == NULL)
                return HINJ_NO_PATH;

        if ((fd = make_socket(path)) < 0)
                return fd;

        if (send(fd, &cpkt, sizeof(cpkt), 0) != sizeof(cpkt)) {
                e = HINJ_BAD_SEND;
                goto clean;
        }

        if (recv(fd, &cpkt, sizeof(cpkt), 0) != sizeof(cpkt)) {
                e = HINJ_BAD_RECV;
                goto clean;
        }

        *voltage = cpkt.voltage;
        *current = cpkt.current;
        *throttle = cpkt.throttle;

clean:
        close(fd);
        return e;
}

int
update_barometer(float *pressure, float *temperature, uint8_t instance)
{
        int fd, e = 0;
        const char *path;
        struct barometer_pkt cpkt = {
                .message_type = HINJ_HIL_BAROMETER,
                .instance = instance,
                .ignore = 0,
                .message_size = sizeof(struct barometer_pkt),
                .pressure = *pressure,
                .temperature = *temperature
        };

        if ((path = make_msg_path()) == NULL)
                return HINJ_NO_PATH;

        if ((fd = make_socket(path)) < 0)
                return fd;

        if (send(fd, &cpkt, sizeof(cpkt), 0) != sizeof(cpkt)) {
                e = HINJ_BAD_SEND;
                goto clean;
        }

        if (recv(fd, &cpkt, sizeof(cpkt), 0) != sizeof(cpkt)) {
                e = HINJ_BAD_RECV;
                goto clean;
        }

        *pressure = cpkt.pressure;
        *temperature = cpkt.temperature;

        if (cpkt.ignore)
                e = HINJ_IGNORE_SENSOR;

clean:
        close(fd);
        return e;
}

int
hinj_update_mode(int mode)
{
        int fd, e = 0;
        const char *path;
        struct mode_pkt pkt = {
                .message_type = HINJ_HIL_MODE,
                .message_size = sizeof(struct mode_pkt),
                .mode = mode
        };

        if ((path = make_msg_path()) == NULL)
                return HINJ_NO_PATH;

        if ((fd = make_socket(path)) < 0)
                return fd;

        if (send(fd, &pkt, sizeof(pkt), 0) != sizeof(pkt)) {
                e = HINJ_BAD_SEND;
                goto clean;
        }


clean:
        close(fd);
        return e;
}

const char*
hinj_strerror(int err)
{
        switch (err) {
        case HINJ_NO_PATH:
                return "HINJ: no suitable message path";
        case HINJ_NO_SOCK:
                return "HINJ: socket() failed";
        case HINJ_NO_CONN:
                return "HINJ: connect() failed";
        case HINJ_BAD_SEND:
                return "HINJ: send() failed";
        case HINJ_BAD_RECV:
                return "HINJ: recv() failed";
        case HINJ_BAD_BIND:
                return "HINJ: bind() failed";
        case HINJ_BAD_LISTEN:
                return "HINJ: listen() failed";
        default:
                return "HINJ: unknown error";
        }
}

static void
init_sync_fd()
{
        struct sockaddr_un sockaddr = {0};
        const char *path;
        int e = 0;

        if ((path = make_sync_path()) == NULL) {
                sync_fd = HINJ_NO_PATH;
                return;
        }

        if ((sync_fd = socket(AF_UNIX, SOCK_STREAM, 0)) == -1) {
                sync_fd = HINJ_NO_SOCK;
                return;
        }

        /* just to be safe, unlink the file path */
        unlink(path);

        strcpy(sockaddr.sun_path, path);
        sockaddr.sun_family = AF_UNIX;

        if (bind(sync_fd, &sockaddr, sizeof(sockaddr))) {
                e = HINJ_BAD_BIND;
                goto clean;
        }

        if (listen(sync_fd, 50)) {
                e = HINJ_BAD_LISTEN;
                goto clean;
        }

        return;

clean:
        close(sync_fd);
        sync_fd = e;
}

static const char*
make_msg_path()
{
        static char *path;
        char *home;
        if (path)
                return path;

        if ((home = getenv("HOME")) == NULL)
                return NULL;

        if (asprintf(&path, "%s/.hardware_controller", home) == -1)
                return NULL;

        return path;
}

static const char*
make_sync_path()
{
        static char *path;
        char *home;
        if (path)
                return path;

        if ((home = getenv("HOME")) == NULL)
                return NULL;

        if (asprintf(&path, "%s/.drone_signal", home) == -1)
                return NULL;

        return path;
}

static int
make_socket(const char *path)
{
        struct sockaddr_un sockaddr = {0};
        int fd;

        if ((fd = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
                return HINJ_NO_SOCK;

        sockaddr.sun_family = AF_UNIX;
        strcpy(sockaddr.sun_path, path);

        if (connect(fd, (struct sockaddr*) &sockaddr, sizeof(sockaddr))) {
                close(fd);
                return HINJ_NO_CONN;
        }

        return fd;
}
