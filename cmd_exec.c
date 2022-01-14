/** 
  ******************************************************************************
  *  @file  cmd_exec.c
  *  @brief Command parser and execution engine.
  *         It is expected that maximum memory access width is 32-bit.
  *         Default width is set to sizeof(unsigned int).
  *         Maybe used with freestanding compiler, no dependencies on libc.
  ******************************************************************************
  *  License: Public domain. The code is provided as is without any warranty.
  *****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <stddef.h>

typedef struct 
{
    uintptr_t addr;
    uint32_t data;
    bool info;
    bool addr_set;
    bool data_set;
    size_t width;
}
operation_t;

typedef enum
{
    PS_KEY,
    PS_ADDR,
    PS_DATA,
}
parser_state_t;

enum
{
    NIBBLE_PER_BYTE = 2,
    NIBBLE_SZ = CHAR_BIT / NIBBLE_PER_BYTE,
    NIBBLE_MASK = (1 << NIBBLE_SZ) - 1,
    DATA_BITS = sizeof(uint32_t) * CHAR_BIT,
    DATA_MASK = DATA_BITS - 1,
};

static inline bool hexdigit(int c)
{
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
}

static inline int isspace(int c)
{
    return (c == ' ') || (c == '\r') || (c == '\n') || (c == '\t');
}

static inline unsigned int to_num(int c)
{
    return (c >= '0' && c <= '9') ? (c - '0') : (c - 'a' + 10);
}

static inline char to_char(int n)
{
    return (n >= 10) ? (n - 10) + 'a' : (n + '0');
}

static inline size_t strncpy(char* dst, const char* src, size_t dstlen)
{
    size_t len = 0;

    while (*src && (len < dstlen))
    {
        *dst++ = *src++;
        ++len;
    }

    return len;
}

static inline void num_to_str(char* buf, uint32_t n, size_t width)
{
    char* p = buf;
    int i = width * CHAR_BIT;

    while ((i -= NIBBLE_SZ) >= 0)
    {
        *p++ = to_char((n >> i) & NIBBLE_MASK);
    }
}

static inline size_t str_to_num(const char* str, uint32_t* data)
{
    uint32_t t = 0;
    size_t i = 0;
        
    for (i = 0; i < sizeof(uint32_t) * NIBBLE_PER_BYTE; ++i)
    {
        if (hexdigit(str[i]))
        {
            t |= to_num(str[i]) << (DATA_BITS - (i + 1) * NIBBLE_SZ);
        }
        else
        {
            break;
        }
    }

    *data = t >> ((DATA_BITS - i * NIBBLE_SZ) & DATA_MASK);
    return i;
}

static size_t parse_key(const char* str, parser_state_t* parser_state, operation_t* cmd)
{
    int offset = 1;

    switch (*str)
    {
    case 'a': cmd->addr_set = true, *parser_state = PS_ADDR; break;
    case 'v': cmd->data_set = true, *parser_state = PS_DATA; break;
    case 't': cmd->info = true, *parser_state = PS_KEY; break;
    case 'b': cmd->width = sizeof(uint8_t), *parser_state = PS_KEY; break;
    case 'w': cmd->width = sizeof(uint16_t), *parser_state = PS_KEY; break;
    case 'd': cmd->width = sizeof(uint32_t), *parser_state = PS_KEY; break;
    default: offset = 0; break;
    };

    return offset;
}

static size_t parse_number(const char* str, parser_state_t* parser_state, operation_t* cmd)
{
    uint32_t data = 0;
    const size_t consumed_chars = str_to_num(str, &data);

    if (*parser_state == PS_ADDR)
    {
        cmd->addr = (uintptr_t) data;
    }
    else
    {
        cmd->data = data;
    }

    *parser_state = PS_KEY;
    return consumed_chars;
}

static int parse_cmdline(const char* str, operation_t* cmd)
{
    parser_state_t parser_state = PS_KEY;

    do
    {
        while (isspace(*str)) 
            ++str;

        str += (parser_state == PS_KEY) ? 
            parse_key(str, &parser_state, cmd) :
            parse_number(str, &parser_state, cmd);
    }
    while (isspace(*str));

    return (*str == '\0');
}

static inline uint32_t do_read(const void* addr, size_t width)
{
    switch (width)
    {
    case sizeof(uint32_t): return *(uint32_t*) addr;
    case sizeof(uint16_t): return *(uint16_t*) addr;
    case sizeof(uint8_t): return *(uint8_t*) addr;
    default: return 0;
    }
}

static inline void do_write(void* addr, uint32_t data, size_t width)
{
    switch (width)
    {
    case sizeof(uint32_t): *(uint32_t*) addr = data; break;
    case sizeof(uint16_t): *(uint16_t*) addr = data; break;
    case sizeof(uint8_t): *(uint8_t*) addr = data; break;
    default: break;
    }
}

static inline size_t execute(const operation_t* cmd, char* buf, size_t len)
{
    size_t response_len = 0;

    if (cmd->addr_set)
    {
        if (cmd->data_set)
        {
            do_write((void*) cmd->addr, cmd->data, cmd->width);
        }
        else
        {
            const uint32_t value = do_read((void*) cmd->addr, cmd->width);
            const size_t datalen = cmd->width * NIBBLE_PER_BYTE;

            if (len >= datalen)
            {
                num_to_str(buf, value, cmd->width);
                response_len = datalen;
            }
        }
    }
    else
    {
        if (cmd->info)
        {
            static const char msg[] = "Device: STM32F103C8Tx; FW: " __DATE__;
            response_len = strncpy(buf, msg, len);
        }
    }

    return response_len;
}

size_t cmd_get_response(char* buf, size_t len)
{
    static const operation_t default_cmd = { 0, 0, false, false, false, sizeof(unsigned int) };
    operation_t cmd = default_cmd;
    const int success = parse_cmdline(buf, &cmd);
    size_t response_len = 0;

    if (success)
    {
        response_len = execute(&cmd, buf, len);
    }
    else
    {
        static const char msg[] = "ERROR";
        response_len = strncpy(buf, msg, len);
    }

    return response_len;
}
