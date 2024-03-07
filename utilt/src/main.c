#include <stdio.h>
#include <time.h>
#include <stdint.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include "rl.h"
#include "pm.h"
#include "log.h"
#include <math.h>

int fd;

#define MAX_BUFFER_SIZE 200
const double GEAR_RATIO = 25.0;
const double STEPS_REV = 200.0 * 32.0;
const double PTR_ENCODER_RATIO = 360.0 / (STEPS_REV * GEAR_RATIO);

uint8_t uart_rx_buffer[MAX_BUFFER_SIZE];
uint8_t uart_rx_cobs_buffer[MAX_BUFFER_SIZE];
uint8_t uart_rx_buffer_index;

uint8_t cobs_zero[1] = {0x00};

#define TILT_ID 2
#define PAN_ID 3
#define ROLL_ID 4

typedef struct
{
    int fd;
    uint8_t uart_tx_buffer[MAX_BUFFER_SIZE];
    uint8_t uart_tx_cobs_buffer[MAX_BUFFER_SIZE];
    uint8_t uart_tx_buffer_index;
} can_t;

can_t canbus;

typedef struct
{
    double PositionValue;
    double VelocityValue;

} in_can_bus_t;

in_can_bus_t in_pan;
in_can_bus_t in_tilt;
in_can_bus_t in_roll;

typedef struct
{
    float TargetPosition;

} out_can_bus_t;

out_can_bus_t out_pan;
out_can_bus_t out_tilt;
out_can_bus_t out_roll;

typedef struct
{
    uint8_t id;
    uint8_t data[8];
    uint8_t len;
} can_message_t;

typedef struct
{
    double pan;
    double tilt;
    double roll;
} all_axis_move_t;

uint16_t cobs_decode(const uint8_t *buffer, size_t length, void *data);
uint16_t cobs_encode(const void *data, uint16_t length, uint8_t *buffer);
void uart_rx_thread();
void decode_frame_to_position(can_message_t can_message);
void set_speed(uint8_t device_id, uint32_t val);
void set_accel(uint8_t device_id, uint32_t val);
void set_decel(uint8_t device_id, uint32_t val);
void set_current(uint8_t device_id, uint32_t val);
void set_current_hold(uint8_t device_id, uint32_t val);
void set_high_speed_mode(uint8_t device_id, uint32_t val);

void init_uart_usb_interface(can_t *can_bus, uint32_t baud, char *serial_port)
{
    char command[100];
    sprintf(command, "stty -F %s %d", serial_port, baud);
    system(command);
    sprintf(command, "stty -F %s -icrnl", serial_port);
    system(command);
    can_bus->fd = open(serial_port, O_RDWR);
    if (can_bus->fd == -1)
    {
        log_error("Error: Unable to open serial port");
        return;
    }
    struct termios options;
    tcgetattr(can_bus->fd, &options);
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    /*this is all the random stuff discovered in order to send packets without weird EOL or other insertions*/
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    options.c_oflag &= ~(ONLCR | OCRNL);
    options.c_iflag &= ~(INLCR | ICRNL);
    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;
    tcsetattr(can_bus->fd, TCSANOW, &options);
    log_info("Serial port: %s opened", serial_port);
}

void send_can_packets(can_t *can_bus, can_message_t *messages, int num_messages)
{
    uint16_t previous_offset = 1;
    can_bus->uart_tx_buffer[0] = num_messages;

    for (int i = 0; i < num_messages; i++)
    {
        can_bus->uart_tx_buffer[0 + previous_offset] = messages[i].id;
        can_bus->uart_tx_buffer[1 + previous_offset] = messages[i].len;
        for (int j = 0; j < messages[i].len; j++)
        {
            can_bus->uart_tx_buffer[j + 2 + previous_offset] = messages[i].data[j];
        }
        previous_offset += messages[i].len + 2;
    }

    if (previous_offset > MAX_BUFFER_SIZE)
    {
        log_error("Error: too many messages to send");
        return;
    }

    uint16_t cobs_length = cobs_encode(can_bus->uart_tx_buffer, previous_offset, can_bus->uart_tx_cobs_buffer);

    write(can_bus->fd, cobs_zero, 1);
    write(can_bus->fd, can_bus->uart_tx_cobs_buffer, cobs_length);
    write(can_bus->fd, cobs_zero, 1);
}

void float_to_buffer(uint8_t reg_value, float value, uint8_t *buffer)
{
    uint8_t *float_buffer = (uint8_t *)&value;
    union
    {
        float f;
        uint8_t b[4];
    } u;
    u.f = value;
    buffer[0] = reg_value;
    for (int i = 1; i < 5; i++)
    {
        buffer[i] = u.b[i];
    }
}

void int32_to_buffer(uint8_t reg_value, int32_t value, uint8_t *buffer)
{
    uint8_t *int_buffer = (uint8_t *)&value;
    union
    {
        int32_t i;
        uint8_t b[4];
    } u;
    u.i = value;
    buffer[0] = reg_value;
    for (int i = 1; i < 5; i++)
    {
        buffer[i] = u.b[i - 1];
    }
}

void uint32_to_buffer(uint8_t reg_value, uint32_t value, uint8_t *buffer)
{
    union
    {
        uint32_t i;
        uint8_t b[4];
    } u;
    u.i = value;
    buffer[0] = reg_value;
    for (int i = 1; i < 5; i++)
    {
        buffer[i] = u.b[i - 1];
    }
}

void reset_bus(can_t *canbus)
{
    char reset[1] = {0};
    write(canbus->fd, &reset, 1);
    write(canbus->fd, &reset, 1);
    write(canbus->fd, &reset, 1);
    usleep(1000000);
}

void send_single_packet(can_t *canbus, uint8_t id, uint8_t len, uint8_t *data)
{
    can_message_t message;
    message.id = id;
    message.len = len;
    for (int i = 0; i < len; i++)
    {
        message.data[i] = data[i];
    }
    can_message_t messages[1];
    messages[0] = message;
    send_can_packets(canbus, messages, 1);
}

void send_all_moves(can_t *canbus, all_axis_move_t *move)
{
    can_message_t messages[3];
    int32_to_buffer(214, (int32_t)(move->pan / PTR_ENCODER_RATIO), messages[0].data);
    messages[0].id = 2;
    messages[0].len = 5;
    int32_to_buffer(214, (int32_t)(move->tilt / PTR_ENCODER_RATIO), messages[1].data);
    messages[1].id = 3;
    messages[1].len = 5;
    int32_to_buffer(214, (int32_t)(move->roll / PTR_ENCODER_RATIO), messages[2].data);
    messages[2].id = 4;
    messages[2].len = 5;
    send_can_packets(canbus, messages, 3);
    //printf("Sent %f %f %f\n", move->roll / PTR_ENCODER_RATIO, move->roll, PTR_ENCODER_RATIO);
}

// super loop
void main()
{

    init_uart_usb_interface(&canbus, 3000000, "/dev/ttyUSB0");
    // sleep for 5 seconds
    // reset_bus(&canbus);

    pthread_t rx_thread;
    pthread_create(&rx_thread, NULL, (void *)uart_rx_thread, NULL);

    all_axis_move_t move;
    move.pan = 1.0;
    move.tilt = 2.0;
    move.roll = 3.0;

    uint8_t buffer[8] = {0x54, 0, 0, 0, 0, 0, 0, 0};
    usleep(10000);
    send_single_packet(&canbus, 0x02, 1, buffer);
    usleep(10000);
    send_single_packet(&canbus, 0x03, 1, buffer);
    usleep(10000);
    send_single_packet(&canbus, 0x04, 1, buffer);
    usleep(10000);


    set_current(2,22);
    set_current(3,22);
    set_current(4,22);

//     set_accel(2, 14000);
//     usleep(10000);
//     set_accel(3, 14000);
// usleep(10000);
//     set_accel(4, 14000);

//     set_speed(2, 20000);
//     set_speed(3, 20000);
//     set_speed(4, 20000);
//     //while (1);
//     set_decel(2, 14000);
//     set_decel(3, 14000);
//     set_decel(4, 14000);
    // set_high_speed_mode(2, 1);
    // set_high_speed_mode(3, 1);
    // set_high_speed_mode(4, 1);

    usleep(1000000);
    log_info("Starting super loop");
    performance_monitor_t pm;
    performance_monitor_init(&pm);
    int32_t counter = 0;
    rate_limiter rl;
    rate_limiter_init(&rl, 1000);
    while (1)
    {
        rate_limiter_limit(&rl);
        performance_monitor_update(&pm);
        performance_monitor_print(&pm);
        // send_single_packet(&canbus, 0x02, 1, buffer);
        move.pan = out_pan.TargetPosition;
        move.tilt = out_tilt.TargetPosition;
        move.roll = out_roll.TargetPosition;
        send_all_moves(&canbus, &move);

        counter += 1;
        float val = sin((float)(counter) / 1000);
        out_pan.TargetPosition = val * 5;
        out_tilt.TargetPosition = val * 5;
        out_roll.TargetPosition = val * 5;
        // print the current position
        log_info("IN Pan: %f Tilt: %f Roll: %f", in_pan.PositionValue, in_tilt.PositionValue, in_roll.PositionValue);

        // log the requested into
        log_info("OUT Pan: %f Tilt: %f Roll: %f", out_pan.TargetPosition, out_tilt.TargetPosition, out_roll.TargetPosition);
        log_info("Fllowing error: %f", out_pan.TargetPosition - in_pan.PositionValue);
    }
}

uint16_t cobs_encode(const void *data, uint16_t length, uint8_t *buffer)
{

    uint8_t *encode = buffer;  // Encoded byte pointer
    uint8_t *codep = encode++; // Output code pointer
    uint8_t code = 1;          // Code value

    for (const uint8_t *byte = (const uint8_t *)data; length--; ++byte)
    {
        if (*byte)
        { // Byte not zero, write it
            *encode++ = *byte, ++code;
        }

        if (!*byte || code == 0xff) // Input is zero or block completed, restart
        {
            *codep = code, code = 1, codep = encode;
            if (!*byte || length)
            {
                ++encode;
            }
        }
    }
    *codep = code; // Write final code value

    return (encode - buffer);
}

uint16_t cobs_decode(const uint8_t *buffer, size_t length, void *data)
{
    const uint8_t *byte = buffer;      // Encoded input byte pointer
    uint8_t *decode = (uint8_t *)data; // Decoded output byte pointer

    for (uint8_t code = 0xff, block = 0; byte < buffer + length; --block)
    {
        if (block)
        { // Decode block byte
            *decode++ = *byte++;
        }
        else
        {
            block = *byte++; // Fetch the next block length
            if (block &&
                (code != 0xff))
            { // Encoded zero, write it unless it's delimiter.
                *decode++ = 0;
            }
            code = block;
            if (!code)
            { // Delimiter code found
                break;
            }
        }
    }

    return (decode - (uint8_t *)data);
}

void uart_rx_thread()
{
    double elapsed_time = 0.0;
    double fps = 0.0;

    while (1)
    {
        uint8_t rx_byte;
        read(canbus.fd, &rx_byte, 1);
        uart_rx_cobs_buffer[uart_rx_buffer_index] = rx_byte;
        uart_rx_buffer_index++;

        // check that the message is long enought and has the start and stop indicators
        if (uart_rx_buffer_index > 2 && uart_rx_cobs_buffer[uart_rx_buffer_index - 1] == 0)
        {
            // one to avoid the start indicator
            // -2 to avoid the start and stop indicators
            cobs_decode(uart_rx_cobs_buffer + 1, uart_rx_buffer_index - 2, uart_rx_buffer);
            uart_rx_buffer_index = 0;
            can_message_t ToCanTx;
            ToCanTx.id = uart_rx_buffer[0];
            ToCanTx.len = uart_rx_buffer[1];
            for (int i = 0; i < ToCanTx.len; i++)
            {
                ToCanTx.data[i] = uart_rx_buffer[i + 2];
            }
            decode_frame_to_position(ToCanTx);
            // print the packet
            //log_info("ID: %d Len: %d Data: %x %x %x %x %x", ToCanTx.id, ToCanTx.len, ToCanTx.data[0], ToCanTx.data[1], ToCanTx.data[2], ToCanTx.data[3], ToCanTx.data[3]);
        }

        // condition if we started to receive half way through another message
        if (uart_rx_cobs_buffer[0] != 0)
        {
            uart_rx_buffer_index = 0;
        }
    }
}

void decode_frame_to_position(can_message_t can_message)
{
    union
    {
        int32_t f;
        uint8_t b[4];
    } u;
    for (int i = 1; i < 5; i++)
    {
        u.b[i - 1] = can_message.data[i];
    }

    // print the value
    //log_info("ID: %d Value: %d", can_message.id, u.f);

    if (can_message.id == 2)
    {
        // pan
        in_pan.PositionValue = (float)u.f * PTR_ENCODER_RATIO;
    }
    else if (can_message.id == 3)
    {
        // tilt
        in_tilt.PositionValue = (float)u.f * PTR_ENCODER_RATIO;
    }
    else if (can_message.id == 4)
    {
        // roll
        in_roll.PositionValue = (float)u.f * PTR_ENCODER_RATIO;
    }
}

void set_speed(uint8_t device_id, uint32_t val)
{
    can_message_t messages[1];
    uint32_to_buffer(10, val, messages[0].data);
    messages[0].id = device_id;
    messages[0].len = 5;
    send_can_packets(&canbus, messages, 1);
}

void set_accel(uint8_t device_id, uint32_t val)
{
    can_message_t new_message[1];
    uint32_to_buffer(11, val, new_message[0].data);
    new_message[0].id = device_id;
    new_message[0].len = 5;
    send_can_packets(&canbus, new_message, 1);
}

void set_decel(uint8_t device_id, uint32_t val)
{
    can_message_t messages[1];
    uint32_to_buffer(14, val, messages[0].data);
    messages[0].id = device_id;
    messages[0].len = 5;
    send_can_packets(&canbus, messages, 1);
}

void set_current(uint8_t device_id, uint32_t val)
{
    can_message_t messages[1];
    uint32_to_buffer(14, val, messages[0].data);
    messages[0].id = device_id;
    messages[0].len = 5;
    send_can_packets(&canbus, messages, 1);
}

void set_current_hold(uint8_t device_id, uint32_t val)
{
    can_message_t messages[1];
    uint32_to_buffer(14, val, messages[0].data);
    messages[0].id = device_id;
    messages[0].len = 5;
    send_can_packets(&canbus, messages, 1);
}

void set_high_speed_mode(uint8_t device_id, uint32_t val)
{
    can_message_t messages[1];
    int32_to_buffer(101, val, messages[0].data);
    messages[0].id = device_id;
    messages[0].len = 5;
    send_can_packets(&canbus, messages, 1);
}

