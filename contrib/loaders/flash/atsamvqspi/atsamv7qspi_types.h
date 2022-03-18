#ifndef ATSAMV7QSPI_TYPES_H_
#define ATSAMV7QSPI_TYPES_H_

#include <stdint.h>

enum atsamv7qspi_executive_commands {
    EXECUTIVE_COMMAND_PROBE,
    EXECUTIVE_COMMAND_ERASE_CHIP,
    EXECUTIVE_COMMAND_ERASE_RANGE,
    EXECUTIVE_COMMAND_WRITE
};

typedef struct {
    uint8_t* start_offset;
    uint32_t n_blocks;
} erase_params_t;

struct fifo_cursors {
    uint8_t* write;
    uint8_t* read;
};

typedef struct {
    struct fifo_cursors* cursors;
    uint8_t* start_offset;
    uint8_t* end_offset;
} fifo_t;

typedef struct {
    fifo_t fifo;
    uint8_t* target;
    uint32_t size;
} write_params_t;

typedef struct {
    uint32_t command;
    union {
        erase_params_t erase;
        write_params_t write;
    } params;
} atsamv7qspi_executive_params_t;

enum atsamv7qspi_executive_error {
    EXECUTIVE_ERROR_OK,
    EXECUTIVE_ERROR_INVALID_OP,
    EXECUTIVE_ERROR_UNKNOWN_DEVICE,
    EXECUTIVE_ERROR_INVALID_OFFSET,
    EXECUTIVE_ERROR_INVALID_SIZE
};

#endif
