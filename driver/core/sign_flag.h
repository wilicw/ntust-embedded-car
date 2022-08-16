#ifndef DRIVER_SIGN_FLAG_H
#define DRIVER_SIGN_FLAG_H

typedef enum {
    SIGN_GO_LEFT,
    SIGN_GO_RIGHT,
    SIGN_NOT_GO,
    SIGN_ONLY_GO,
    SIGN_ONLY_LEFT,
    SIGN_ONLY_RIGHT,
    SIGN_STOP_LINE,
    SIGN_STOP_PIC,
    SIGN_NO_LEFT,
    SIGN_NO_RIGHT,
    SIGN_NO_STOP
} sign_flag_t;

typedef enum {
    CMD_HALT,
    CMD_GO,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_TURN,
} sign_cmd_t;

#endif  // DRIVER_SIGN_FLAG_H
