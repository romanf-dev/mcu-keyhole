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

enum opcode_t
{
    OP_READ,
    OP_WRITE,
    OP_WAIT_UNTIL,
    OP_INFO,
};

struct operation_t
{
    enum opcode_t op;
    size_t width;
    uint32_t data[2];
};

struct cmd_descr_t
{
    enum opcode_t op;
    size_t args_num;
    bool modifier;
};

enum
{
    NIBBLE_PER_BYTE = 2,
    NIBBLE_SZ = CHAR_BIT / NIBBLE_PER_BYTE,
    NIBBLE_MASK = (1 << NIBBLE_SZ) - 1,
    DATA_BITS = sizeof(uint32_t) * CHAR_BIT,
    DATA_MASK = DATA_BITS - 1,
    CMD_SEPARATOR = '|',
    CMD_STORAGE = 5,
};

static const char g_info[] = "STM32F103C8 FW ver: " __DATE__;
static const uint32_t g_default_wait_tout = 50000;

static inline bool hexdigit(const int c)
{
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
}

static inline unsigned int to_num(const int c)
{
    return (c >= '0' && c <= '9') ? (c - '0') : (c - 'a' + 10);
}

static inline char to_char(const int n)
{
    return (n >= 10) ? (n - 10) + 'a' : (n + '0');
}

static inline bool isspace(const int c)
{
    return ((c == ' ') || (c == '\t') || (c == '\r') || (c == '\n'));
}

static inline const char* skip_spaces(const char str[static 1])
{
    while (isspace(*str))
    {
        ++str;
    }

    return str;
}

static void append_string(char s[restrict static 1], const char resp[restrict static 1])
{
    while (*resp != '\0')
    {
        *s++ = *resp++;
    }

    *s = '\0';
}

static inline size_t num_to_str(const uint32_t n, const size_t width, char buf[static width * NIBBLE_PER_BYTE])
{
    char* p = buf;
    int i = width * CHAR_BIT;

    while ((i -= NIBBLE_SZ) >= 0)
    {
        *p++ = to_char((n >> i) & NIBBLE_MASK);
    }

    return width * NIBBLE_PER_BYTE;
}

static inline const char* str_to_num(const char str[static 1], uint32_t* n)
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

    *n = t >> ((DATA_BITS - i * NIBBLE_SZ) & DATA_MASK);
    return i ? (str + i) : (void*)0;
}

static inline size_t get_width(const int c)
{
    size_t width = 0;

    switch (c)
    {
    case 'b': width = sizeof(uint8_t); break;
    case 'w': width = sizeof(uint16_t); break;
    case 'd': width = sizeof(uint32_t); break;
    default: break;
    }

    return width;
}

static inline const struct cmd_descr_t* get_cmd_descr(const int c)
{
    static const struct cmd_descr_t cmds[] = 
    {
        [OP_READ] = { .op = OP_READ, .args_num = 1, .modifier = true},
        [OP_WRITE] = { .op = OP_WRITE, .args_num = 2, .modifier = true},
        [OP_INFO] = { .op = OP_INFO, .args_num = 0, .modifier = false},
        [OP_WAIT_UNTIL] = { .op = OP_WAIT_UNTIL, .args_num = 2, .modifier = true}
    };

    const struct cmd_descr_t* descr = (void*)0;

    switch (c)
    {
    case 'r': descr = &cmds[OP_READ]; break;
    case 'w': descr = &cmds[OP_WRITE]; break;
    case 'i': descr = &cmds[OP_INFO]; break;
    case 'u': descr = &cmds[OP_WAIT_UNTIL]; break;
    default: break;
    }

    return descr;
}

static inline const char* parse_key(const char str[const static 1], size_t* arg, struct operation_t* cmd)
{
    size_t cmd_len = 0;
    const char* s = (void*)0;
    const struct cmd_descr_t* const descr = get_cmd_descr(*str);

    if (descr)
    {
        cmd->op = descr->op;
        cmd->width = sizeof(unsigned int);
        *arg = descr->args_num;
        cmd_len = sizeof(*str);

        if (descr->modifier)
        {
            const size_t width = get_width(str[cmd_len]);

            if (width)
            {
                cmd_len += sizeof(*str);
                cmd->width = width;
            }
        }

        s = str + cmd_len;
    }

    return s;
}

static const char* parse_cmd(const char str[const static 1], struct operation_t* cmd)
{
    size_t args = 0;
    const char* s = skip_spaces(str);

    s = parse_key(s, &args, cmd);

    if (s != (void*)0)
    {
        for (size_t i = 0; i < args; ++i)
        {
            const char* token = skip_spaces(s);

            if (token == s)
            {
                s = (void*)0;
                break;
            }

            s = str_to_num(token, &cmd->data[i]);

            if (s == (void*)0)
            {
                break;
            }
        }        
    }

    return s;
}

static inline uint32_t do_read(const uintptr_t addr, const size_t width)
{
    uint32_t value = 0;

    switch (width)
    {
    case sizeof(uint32_t): value = *(volatile uint32_t*) addr; break;
    case sizeof(uint16_t): value = *(volatile uint16_t*) addr; break;
    case sizeof(uint8_t): value = *(volatile uint8_t*) addr; break;
    default: break;
    }

    return value;
}

static inline void do_write(const uintptr_t addr, const uint32_t n, const size_t width)
{
    switch (width)
    {
    case sizeof(uint32_t): *(volatile uint32_t*) addr = n; break;
    case sizeof(uint16_t): *(volatile uint16_t*) addr = n; break;
    case sizeof(uint8_t): *(volatile uint8_t*) addr = n; break;
    default: break;
    }
}

static inline size_t do_info(char buf[const static sizeof(g_info)])
{
    const size_t len = sizeof(g_info) - 1;

    for (size_t i = 0; i < len; ++i)
    {
        buf[i] = g_info[i];
    }

    return len;
}

static inline uint32_t do_wait(const uintptr_t addr, const uint32_t n, const size_t width, bool* stop)
{
    const union
    {
        uint32_t value;

        struct
        {
            uint32_t bit:5;
            uint32_t val:1;
            uint32_t stop:1;
            uint32_t reserved:1;
            uint32_t timeout:24;        
        } 
        field;
    }
    wait_id = { n };
    
    _Static_assert(sizeof(wait_id) == sizeof(uint32_t), "");

    const uint32_t mask = (UINT32_C(1) << wait_id.field.bit);
    const uint32_t expect = (wait_id.field.val == 1) ? mask : 0;
    const volatile uint8_t* const pb = (volatile void*) addr;
    const volatile uint16_t* const pw = (volatile void*) addr;
    const volatile uint32_t* const pd = (volatile void*) addr;
    const uint32_t timeout = wait_id.field.timeout ? wait_id.field.timeout : g_default_wait_tout;
    uint32_t last_val = 0;
    uint32_t i = 0;

    switch (width)
    {
    case sizeof(uint8_t): while ((((last_val = *pb) & mask) != expect) && (i++ < timeout)); break;
    case sizeof(uint16_t): while ((((last_val = *pw) & mask) != expect) && (i++ < timeout)); break;
    case sizeof(uint32_t): while ((((last_val = *pd) & mask) != expect) && (i++ < timeout)); break;
    default: break;
    }

    const bool timeout_reached = ((last_val & mask) != expect);

    if (wait_id.field.stop && timeout_reached)
    {
        *stop = true;
    }

    return last_val;
}

static inline size_t run_cmd(const struct operation_t* cmd, char buf[const static 1], bool* stop)
{
    size_t offset = 0;

    switch (cmd->op)
    {
    case OP_READ: 
    {
        const uint32_t value = do_read(cmd->data[0], cmd->width);
        offset = num_to_str(value, cmd->width, buf);
        break;
    }
    case OP_WRITE: 
    {
        do_write(cmd->data[0], cmd->data[1], cmd->width);
        break;
    }
    case OP_INFO:
    {
        offset = do_info(buf);
        break;
    }
    case OP_WAIT_UNTIL:
    {
        const uint32_t value = do_wait(cmd->data[0], cmd->data[1], cmd->width, stop);
        offset = num_to_str(value, cmd->width, buf);        
        break;
    }
    }

    return offset;
}

static inline size_t get_response_len(const struct operation_t* cmd)
{
    const size_t width = (cmd->op != OP_INFO) ? cmd->width : 1;

    static const size_t resp_len[] = 
    {
        [OP_READ] = NIBBLE_PER_BYTE,
        [OP_WRITE] = 0,
        [OP_INFO] = sizeof(g_info) - 1,
        [OP_WAIT_UNTIL] = NIBBLE_PER_BYTE
    };

    return  resp_len[cmd->op] * width;
}

static inline char* run_seq(
    const size_t size, 
    const struct operation_t cmd[const static size], 
    size_t len, 
    char buf[static len])
{
    bool stop = false;

    for (unsigned int i = 0; i < size; ++i)
    {
        const bool non_last = (i != size - 1);
        const size_t req_buf_sz = get_response_len(&cmd[i]) + non_last;

        if (req_buf_sz > len)
        {
            buf = (void*)0;
            break;
        }

        const size_t resp_len = run_cmd(&cmd[i], buf, &stop);

        buf += resp_len;
        len -= resp_len;

        if (stop)
        {
            break;
        }

        if (non_last)
        {
            *buf++ = CMD_SEPARATOR;
            --len;
        }
    }

    return buf;
}

static inline size_t execute(const size_t len, char buf[const static len])
{
    struct operation_t cmd = {0};
    static struct operation_t ops_arr[CMD_STORAGE] = {0};
    unsigned int i = 0;
    const char* s = buf;
    size_t response_len = 0;

    static const char err_syntax[] = "#ERROR\r\n";
    static const char err_input_too_long[] = "#SEQTOOLONG\r\n";
    static const char err_out_too_long[] = "#RESPTOOLONG\r\n";
    static const char err_unexp_end[] = "#ENDOFSTRING\r\n";
    static const char crlf[] = "\r\n";

    for (;;)
    {
        s = parse_cmd(s, &cmd);

        if (s == (void*)0)
        {
            append_string(buf, err_syntax);
            response_len = sizeof(err_syntax);
            break;
        }

        if (i < (sizeof(ops_arr) / sizeof(ops_arr[0])))
        {
            ops_arr[i++] = cmd;
        }
        else 
        {
            append_string(buf, err_input_too_long);
            response_len = sizeof(err_input_too_long);
            break;
        }

        s = skip_spaces(s);

        if (*s == CMD_SEPARATOR)
        {
            ++s;
        }
        else if (*s == '\0')
        {
            char* const reply = run_seq(i, ops_arr, len - sizeof(crlf), buf);

            if (reply == (void*)0)
            {
                append_string(buf, err_out_too_long);
                response_len = sizeof(err_out_too_long);
            }
            else
            {
                const size_t used_len = reply - buf;
                append_string(reply, crlf);
                response_len = used_len + sizeof(crlf);
            }

            break;
        }
        else
        {
            append_string(buf, err_unexp_end);
            response_len = sizeof(err_unexp_end);
            break;
        }
    }

    return response_len;
}

size_t cmd_get_response(char* buf, size_t len)
{
    return execute(len, buf);
}

