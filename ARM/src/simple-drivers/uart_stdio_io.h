#ifndef _uart_stdio_io_h
#define _uart_stdio_io_h

enum {
    UART_STDIO_IO_MODE_COOKED = 0,
    UART_STDIO_IO_MODE_RAW = 1
};

typedef int (*UART_STDIO_IO_STDOUT)(unsigned char *buffer, int len, void *usr);
typedef int (*UART_STDIO_IO_STDIN)(unsigned char *buffer, int len, void *usr);
typedef void (*UART_STDIO_IO_SET_READ_TIMEOUT)(int timeout, void *usr);
typedef int (*UART_STDIO_IO_SET_MODE)(int mode, void *usr);

typedef struct UART_STDIO_IO {
    UART_STDIO_IO_STDOUT out;
    UART_STDIO_IO_STDIN in;
    UART_STDIO_IO_SET_READ_TIMEOUT set_read_timeout;
    UART_STDIO_IO_SET_MODE set_mode;
    void *usr;
} UART_STDIO_IO;

#endif
