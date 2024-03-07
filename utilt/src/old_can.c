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

int fd;

#define MAX_BUFFER_SIZE 200
#define GEAR_RATIO 25
#define STEPS_REV 200 * 32
#define PTR_ENCODER_RATIO 2

uint8_t uart_rx_buffer[MAX_BUFFER_SIZE];
uint8_t uart_rx_cobs_buffer[MAX_BUFFER_SIZE];
uint8_t uart_rx_buffer_index;

uint8_t cobs_zero[1] = {0x00};

#define TILT_ID 2
#define PAN_ID 3
#define ROLL_ID 4

typedef struct
{
    int32_t PositionValue;
    int32_t VelocityValue;

} in_can_bus_t;

typedef struct
{
    int32_t TargetPosition;

} out_can_bus_t;

typedef struct
{
    uint8_t id;
    uint8_t data[8];
    uint8_t len;
} can_message_t;

typedef struct
{
    float pan;
    float tilt;
    float roll;
} all_axis_move_t;

typedef struct
{
    int fd;
    uint8_t uart_tx_buffer[MAX_BUFFER_SIZE];
    uint8_t uart_tx_cobs_buffer[MAX_BUFFER_SIZE];
    uint8_t uart_tx_buffer_index;
} can_t;

uint16_t cobs_decode(const uint8_t *buffer, size_t length, void *data);
uint16_t cobs_encode(const void *data, uint16_t length, uint8_t *buffer);
void uart_rx_thread();

void init_uart_usb_interface(can_t *can_bus, uint32_t baud, char* serial_port)
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
    //print the packet
    printf("Packet: ");
    for (int i = 0; i < previous_offset; i++)
    {
        printf("%x ", can_bus->uart_tx_buffer[i]);
    }
    printf("\n");

    uint16_t cobs_length = cobs_encode(can_bus->uart_tx_buffer, previous_offset, can_bus->uart_tx_cobs_buffer);

    //print the cobs buffer
    printf("COBS: ");
    for (int i = 0; i < cobs_length; i++)
    {
        printf("%x ", can_bus->uart_tx_cobs_buffer[i]);
    }
    printf("\n");

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
    float_to_buffer(0x2e, move->pan, messages[0].data);
    messages[0].id = 4;
    messages[0].len = 5;
    float_to_buffer(0x2e, move->tilt, messages[1].data);
    messages[1].id = 4;
    messages[1].len = 5;
    float_to_buffer(0x2e, move->roll, messages[2].data);
    messages[2].id = 4;
    messages[2].len = 5;
    send_can_packets(canbus, messages, 3);
}

// super loop
void main()
{
    can_t canbus;
    init_uart_usb_interface(&canbus, 1000000, "/dev/ttyUSB1");
    reset_bus(&canbus);

    pthread_t rx_thread;
    pthread_create(&rx_thread, NULL, (void *)uart_rx_thread, NULL);

    all_axis_move_t move;
    move.pan = 1.0;
    move.tilt = 2.0;
    move.roll = 3.0;

    performance_monitor_t pm;
    performance_monitor_init(&pm);

    rate_limiter rl;
    rate_limiter_init(&rl, 10);
    
    uint8_t buffer[8] = {0x21, 0, 0, 0, 0, 0, 0, 0};

    while (1)
    {
        rate_limiter_limit(&rl);
        performance_monitor_update(&pm);
        performance_monitor_print(&pm);
        //send_single_packet(&canbus, 0x02, 1, buffer);
        send_all_moves(&canbus, &move);
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
    printf("Starting RX thread\n");

    while (1)
    {
        uint8_t rx_byte;
        read(fd, &rx_byte, 1);
        printf("%c", rx_byte);
    }
}


// double get_time()
// {
//     struct timespec ts;
//     clock_gettime(CLOCK_MONOTONIC, &ts);
//     return ts.tv_sec + ts.tv_nsec * 1e-9;
// }

// void uart_rx_thread()
// {
//     // watch for data on uart and print when a full cobs packet is received
//     // make a timer to p
//     int frame_count = 0;
//     double last_time = get_time();
//     double elapsed_time = 0.0;
//     double fps = 0.0;

//     while (1)
//     {
//         uint8_t rx_byte;
//         read(fd, &rx_byte, 1);
//         uart_rx_cobs_buffer[uart_rx_buffer_index] = rx_byte;
//         uart_rx_buffer_index++;
//         // printf("rx_byte: %x\n", rx_byte);

//         // check that the message is long enought and has the start and stop indicators
//         if (uart_rx_buffer_index > 2 && uart_rx_cobs_buffer[uart_rx_buffer_index - 1] == 0)
//         {
//             // one to avoid the start indicator
//             // -2 to avoid the start and stop indicators
//             cobsDecode(uart_rx_cobs_buffer + 1, uart_rx_buffer_index - 2, uart_rx_buffer);
//             uart_rx_buffer_index = 0;
//             CanMessage ToCanTx;
//             ToCanTx.id = uart_rx_buffer[0];
//             ToCanTx.len = uart_rx_buffer[1];
//             for (int i = 0; i < ToCanTx.len; i++)
//             {
//                 ToCanTx.data[i] = uart_rx_buffer[i + 2];
//             }

//             // printf("ID: %x\n", ToCanTx.id);
//             transmit_flag = 1;
//             frame_count++;
//         }

//         // condition if we started to receive half way through another message
//         if (uart_rx_cobs_buffer[0] != 0)
//         {
//             uart_rx_buffer_index = 0;
//         }

//         elapsed_time = get_time() - last_time;
//         if (elapsed_time >= 1.0)
//         {
//             fps = frame_count / elapsed_time;
//             printf("FPS: %.2f\n", fps);
//             frame_count = 0;
//             last_time = get_time();
//         }
//     }
// }

// void main()
// {


// }
