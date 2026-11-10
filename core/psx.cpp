// PlayStation core (psxtang) for the Tang Console 138K.
//
// psxtang does not use TangCore's iosys protocol: its AE350 RISC-V CPU runs its own firmware
// (menus on its own OSD, CD images and memory cards on the SD card, USB pads on the FPGA's ports).
// The BL616 only starts it:
//   1. program cores/<board>/psxtang.bin over JTAG (this replaces the monitor core),
//   2. send the AE350 firmware (psxtang-firmware.bin, next to the core) to the core's UART
//      loader, which writes it to DDR3 at address 0, and read it back to check it,
//   3. hand the SD card to the FPGA (GPIO16 low), then let the AE350 out of reset.
// From then on the BL616 stays off the SD card and sends nothing more on the UART: the loader
// would take a stray 0xAA frame as a command (CPU reset, DDR3 write). The UART stays configured,
// so its TX line idles high instead of floating.
//
// UART loader frames (src/uart_loader_adc_dual.sv in psxtang): 0xAA, length of the parameters
// (big endian, 16 bits), command, parameters.
//   0x11 [0 / 1]                     hold / release the AE350 reset
//   0x12 [addr LE32] [data]          write DDR3, data a multiple of 16 bytes
//   0x14 [addr LE32] [bytes LE16]    read DDR3 (up to 1024 bytes), answered by
//   0x94 [addr LE32] [data] [timeouts]

#include <string>
#include <string.h>

extern "C" {
#include "bflb_gpio.h"
#include "bflb_uart.h"
#include "bflb_mtimer.h"
}

#include "utils.h"
#include "cores.h"
#include "overlay.h"
#include "programmer.h"

#define PSX_CORE_FILE       "psxtang.bin"
#define PSX_FIRMWARE_FILE   "psxtang-firmware.bin"
#define PSX_FIRMWARE_MAX    (16u * 1024u * 1024u)
#define PSX_FIRMWARE_ADDR   0x00000000u

// The loader's rate (psxtang top.sv: uart_loader Baud); TangCore talks to its cores at 2 Mbaud
// too, so the switch below is only there in case the two ever differ.
#define PSX_LOADER_BAUD     2000000
#define TANGCORE_BAUD       2000000

#define LOADER_READY_MS     5000    // DDR3 calibration holds the loader in reset after programming
#define WRITE_CHUNK         512
#define READ_CHUNK          1024
#define VERIFY_TRIES        3

#define CMD_RESET           0x11
#define CMD_WRITE           0x12
#define CMD_READ            0x14
#define CMD_READ_REPLY      0x94

extern TaskHandle_t uart1_rx_task_handle;
extern const char *drv;

static uint8_t chunk[READ_CHUNK];
static uint8_t readback[READ_CHUNK];

static void put(uint8_t b)
{
    bflb_uart_putchar(uart1_dev, b);
}

static void send_frame(uint8_t cmd, const uint8_t *params, uint16_t len)
{
    put(0xAA);
    put(len >> 8);
    put(len & 0xFF);
    put(cmd);
    for (uint16_t i = 0; i < len; i++)
        put(params[i]);
}

static void send_reset(bool hold)
{
    uint8_t p = hold ? 0 : 1;
    send_frame(CMD_RESET, &p, 1);
}

static void send_read(uint32_t addr, uint16_t bytes)
{
    uint8_t p[6] = { (uint8_t)addr, (uint8_t)(addr >> 8), (uint8_t)(addr >> 16), (uint8_t)(addr >> 24),
                     (uint8_t)bytes, (uint8_t)(bytes >> 8) };
    send_frame(CMD_READ, p, sizeof(p));
}

static void drain_rx(void)
{
    while (bflb_uart_rxavailable(uart1_dev))
        bflb_uart_getchar(uart1_dev);
}

// Drop whatever is still arriving (the rest of a reply that was cut short) until the line has
// been quiet for 2 ms.
static void drain_quiet(void)
{
    uint64_t last = bflb_mtimer_get_time_ms();
    while (bflb_mtimer_get_time_ms() - last < 2) {
        if (bflb_uart_rxavailable(uart1_dev)) {
            bflb_uart_getchar(uart1_dev);
            last = bflb_mtimer_get_time_ms();
        }
    }
}

static bool get_byte(uint8_t *b, uint64_t deadline)
{
    while (!bflb_uart_rxavailable(uart1_dev)) {
        if (bflb_mtimer_get_time_ms() > deadline)
            return false;
    }
    *b = (uint8_t)bflb_uart_getchar(uart1_dev);
    return true;
}

// Wait for the 0x94 reply to a read of `bytes` at `addr`; other frames are skipped.
static bool get_read_reply(uint32_t addr, uint8_t *out, uint16_t bytes, uint32_t timeout_ms)
{
    uint64_t deadline = bflb_mtimer_get_time_ms() + timeout_ms;
    for (;;) {
        uint8_t b, lh, ll, cmd;
        do {
            if (!get_byte(&b, deadline)) return false;
        } while (b != 0xAA);
        if (!get_byte(&lh, deadline) || !get_byte(&ll, deadline) || !get_byte(&cmd, deadline))
            return false;
        uint16_t len = (uint16_t)(lh << 8 | ll);
        if (cmd != CMD_READ_REPLY || len != bytes + 5u) {
            for (uint16_t i = 0; i < len; i++)         // some other frame: skip it
                if (!get_byte(&b, deadline)) return false;
            continue;
        }
        uint32_t a = 0;
        for (int i = 0; i < 4; i++) {
            if (!get_byte(&b, deadline)) return false;
            a |= (uint32_t)b << (8 * i);
        }
        for (uint16_t i = 0; i < bytes; i++)
            if (!get_byte(&out[i], deadline)) return false;
        uint8_t timeouts;
        if (!get_byte(&timeouts, deadline)) return false;
        return a == addr && timeouts == 0;
    }
}

// The loader comes out of reset when DDR3 calibration is done: ask until it answers.
static bool wait_loader(void)
{
    uint64_t start = bflb_mtimer_get_time_ms();
    while (bflb_mtimer_get_time_ms() - start < LOADER_READY_MS) {
        drain_rx();
        send_read(PSX_FIRMWARE_ADDR, 16);
        if (get_read_reply(PSX_FIRMWARE_ADDR, readback, 16, 100))
            return true;
    }
    return false;
}

static bool send_firmware(FIL *f, uint32_t size)
{
    uint32_t off = 0;
    while (off < size) {
        UINT n = 0;
        uint32_t want = (size - off) < WRITE_CHUNK ? (size - off) : WRITE_CHUNK;
        if (f_read(f, chunk, want, &n) != FR_OK || n != want)
            return false;
        uint16_t len = (uint16_t)((n + 15u) & ~15u);   // whole 16 byte DDR3 words, 0xFF filled
        memset(chunk + n, 0xFF, len - n);
        uint32_t a = PSX_FIRMWARE_ADDR + off;
        put(0xAA);
        put((len + 4) >> 8);
        put((len + 4) & 0xFF);
        put(CMD_WRITE);
        put(a); put(a >> 8); put(a >> 16); put(a >> 24);
        for (uint16_t i = 0; i < len; i++)
            put(chunk[i]);
        off += n;
    }
    return true;
}

static bool verify_firmware(FIL *f, uint32_t size)
{
    if (f_lseek(f, 0) != FR_OK) return false;
    uint32_t off = 0;
    while (off < size) {
        UINT n = 0;
        uint32_t want = (size - off) < READ_CHUNK ? (size - off) : READ_CHUNK;
        if (f_read(f, chunk, want, &n) != FR_OK || n != want)
            return false;
        uint16_t len = (uint16_t)((n + 15u) & ~15u);
        uint32_t a = PSX_FIRMWARE_ADDR + off;
        // At 2 Mbaud a byte comes every 5 us and the RX FIFO holds 32: a long interrupt can lose
        // part of a reply. Ask again; a real difference in DDR3 is there every time.
        bool ok = false;
        for (int attempt = 0; attempt < VERIFY_TRIES && !ok; attempt++) {
            drain_quiet();
            send_read(a, len);
            ok = get_read_reply(a, readback, len, 1000) && memcmp(chunk, readback, n) == 0;
        }
        if (!ok)
            return false;
        off += n;
    }
    return true;
}

// Something went wrong after the monitor core was replaced: put it back to show the message.
static void back_to_monitor(const char *msg)
{
    bflb_uart_feature_control(uart1_dev, UART_CMD_SET_BAUD_RATE, TANGCORE_BAUD);
    fpga_tx_enable(true);
    std::string monitor;
    if (find_core_for_board(monitor, "monitor.bin"))
        fpga_program(monitor.c_str());
    drain_rx();
    vTaskResume(uart1_rx_task_handle);
    delay(500);
    overlay(1);
    overlay_message(msg, 1);
    overlay_status("%s", msg);
}

bool psx_boot(void)
{
    std::string core_file, fw_file;
    if (!find_core_for_board(core_file, PSX_CORE_FILE)) {
        overlay_message("cores/" PSX_CORE_FILE " not found", 1);
        return false;
    }
    if (!find_core_for_board(fw_file, PSX_FIRMWARE_FILE)) {
        overlay_message("cores/" PSX_FIRMWARE_FILE " not found", 1);
        return false;
    }
    uint32_t fw_size = get_file_size(fw_file.c_str());
    if (fw_size == 0 || fw_size > PSX_FIRMWARE_MAX) {
        overlay_message(PSX_FIRMWARE_FILE " has a bad size", 1);
        return false;
    }
    static USB_NOCACHE_RAM_SECTION FIL ffw;
    if (f_open(&ffw, fw_file.c_str(), FA_READ) != FR_OK) {
        overlay_message("Cannot open " PSX_FIRMWARE_FILE, 1);
        return false;
    }
    overlay_status("PlayStation: %s, firmware %u bytes", core_file.c_str(), (unsigned)fw_size);

    // From here on only this code may use the UART: the monitor core's frames mean something
    // else to the PSX loader.
    // TangCore's own frames stop before programming: its messages after the JTAG load would reach
    // the new core's loader.
    vTaskSuspend(uart1_rx_task_handle);
    fpga_tx_enable(false);
    if (!fpga_program(core_file.c_str())) {
        f_close(&ffw);
        back_to_monitor("Programming psxtang failed");
        return false;
    }
    bflb_uart_feature_control(uart1_dev, UART_CMD_SET_BAUD_RATE, PSX_LOADER_BAUD);

    const char *err = NULL;
    if (!wait_loader())
        err = "psxtang loader did not answer";
    else {
        send_reset(true);
        if (!send_firmware(&ffw, fw_size))
            err = "Reading the firmware failed";
        else if (!verify_firmware(&ffw, fw_size))
            err = "Firmware check failed";
    }
    f_close(&ffw);
    if (err) {
        back_to_monitor(err);
        return false;
    }

    // The SD card goes to the FPGA (the AE350 mounts it). Nothing on this side may use it again.
    f_unmount(drv);
    bflb_gpio_reset(gpio_dev, GPIO_PIN_16);
    delay(10);
    send_reset(false);
    return true;
}
