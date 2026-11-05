"""Run the real UART6 adapter and RV2 C parser against a host HAL/OLED fixture."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from rescue_vision.simple_protocol import SimpleObject, SimpleObservation, encode


ROOT = Path(__file__).resolve().parents[2]

HAL = r'''
#ifndef MOCK_HAL_H
#define MOCK_HAL_H
#include <stdint.h>
typedef struct { void *Instance; } UART_HandleTypeDef;
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
#define USART6 ((void *)6)
#define __DMB() ((void)0)
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
#define __HAL_UART_CLEAR_OREFLAG(handle) ((void)(handle))
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
uint32_t HAL_GetTick(void);
#endif
'''

OLED = r'''
#include <stdint.h>
void OLED_ClearArea(int16_t, int16_t, uint8_t, uint8_t);
void OLED_Printf1(int16_t, int16_t, char *, ...);
void OLED_Update(void);
'''

HARNESS = r'''
#include "vision_uart6.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

UART_HandleTypeDef huart6 = { USART6 };
static uint32_t tick;
static uint8_t *receive_buffer;
static int receive_failed;
static char rows[8][64];

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *buffer, uint16_t size) {
    (void)h;
    (void)size;
    if (receive_failed) return 1;
    receive_buffer = buffer;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h) {
    (void)h;
    receive_buffer = NULL;
    return HAL_OK;
}
uint32_t HAL_GetTick(void) { return tick; }
void OLED_ClearArea(int16_t x, int16_t y, uint8_t width, uint8_t height) {
    (void)x; (void)y; (void)width; (void)height;
    memset(rows, 0, sizeof(rows));
}
void OLED_Printf1(int16_t x, int16_t y, char *format, ...) {
    va_list args;
    (void)x;
    if (y < 0 || y >= 64 || y % 8) abort();
    va_start(args, format);
    vsnprintf(rows[y / 8], sizeof(rows[0]), format, args);
    va_end(args);
}
void OLED_Update(void) {
    unsigned row;
    for (row = 0; row < 8; row++) puts(rows[row]);
    puts("SNAPSHOT_END");
}
int main(void) {
    char command[4096];
    vision_uart6_init();
    while (fgets(command, sizeof(command), stdin)) {
        if (command[0] == 'T') tick = (uint32_t)strtoul(command + 2, NULL, 10);
        else if (command[0] == 'P') vision_uart6_process();
        else if (command[0] == 'S') vision_uart6_oled_task();
        else if (command[0] == 'E') vision_uart6_error(&huart6);
        else if (command[0] == 'Q') receive_failed = atoi(command + 2);
        else if (command[0] == 'H') {
            const char *p;
            for (p = command + 2; p[0] && p[0] != '\n' && p[1] && p[1] != '\n'; p += 2) {
                char token[3] = { p[0], p[1], 0 };
                if (receive_buffer) {
                    *receive_buffer = (uint8_t)strtoul(token, NULL, 16);
                    vision_uart6_rx_complete(&huart6);
                }
            }
        }
    }
    return 0;
}
'''


def packet(source="upper", frame=1, age=10, objects=None, status="ok"):
    if objects is None:
        objects = (SimpleObject("green", 450, True, False, -12.25),)
    return encode(SimpleObservation(source, frame, age, status, objects))


def feed(data, tick=100):
    return [f"T {tick}", *["H " + data[i:i+240].hex() for i in range(0, len(data), 240)]]


@unittest.skipUnless(shutil.which("gcc"), "Native GCC is required for C adapter regression checks")
class Uart6Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix=".uart6-", dir=Path(__file__).parent)
        directory = Path(cls.temporary.name)
        (directory / "stm32f4xx_hal.h").write_text(HAL, encoding="ascii")
        (directory / "OLED.h").write_text(OLED, encoding="ascii")
        (directory / "fixture.c").write_text(HARNESS, encoding="ascii")
        cls.executable = directory / "uart6_test.exe"
        subprocess.run([shutil.which("gcc"), "-std=c99", "-Wall", "-Wextra", "-Werror",
                        "-I" + str(directory), "-I" + str(ROOT / "project/Core/Inc"),
                        "-I" + str(ROOT / "perception/stm32"), str(directory / "fixture.c"),
                        str(ROOT / "project/Core/Src/vision_uart6.c"),
                        str(ROOT / "perception/stm32/vision_simple.c"), "-lm", "-o", str(cls.executable)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def run_commands(self, commands):
        result = subprocess.run([str(self.executable)], input="\n".join(commands) + "\n",
                                text=True, capture_output=True, check=True)
        return [part.strip().splitlines() for part in result.stdout.split("SNAPSHOT_END") if part.strip()]

    def test_upper_and_lower_are_displayed_independently(self):
        lower = packet("lower", objects=(SimpleObject("green", -1, False, True, None),))
        rows = self.run_commands([*feed(packet()), *feed(lower), "P", "S"])[0]
        self.assertEqual(rows[:4], ["UP GREEN 1/1 mm", "X:-95 Y:+440", "A:-12.25deg", "D:450mm"])
        self.assertEqual(rows[4:8], ["LOW GREEN 1/1 mm", "X:NA Y:NA", "A:NA", "D:NA"])

    def test_estimates_are_marked_and_missing_angle_stays_na(self):
        objects = (SimpleObject("green", 200, False, False, 0.),)
        rows = self.run_commands([*feed(packet(objects=objects)), "P", "S"])[0]
        self.assertEqual(rows[1:4], ["X:~+0 Y:~+200", "A:~+0.00deg", "D:~200mm"])
        objects = (SimpleObject("green", 200, True, False, None),)
        rows = self.run_commands([*feed(packet(objects=objects)), "P", "S"])[0]
        self.assertEqual(rows[1:4], ["X:NA Y:NA", "A:NA", "D:200mm"])

    def test_valid_values_become_estimates_at_150_and_disappear_at_300(self):
        snapshots = self.run_commands([*feed(packet()), "P", "T 240", "S", "T 390", "S"])
        self.assertEqual(snapshots[0][:4], ["UP GREEN 1/1 mm", "X:~-95 Y:~+440", "A:~-12.25deg", "D:~450mm"])
        self.assertEqual(snapshots[1][:4], ["UP STALE", "X:NA Y:NA", "A:NA", "D:NA"])

    def test_queue_delay_is_included_in_age(self):
        rows = self.run_commands([*feed(packet()), "T 260", "P", "S"])[0]
        self.assertEqual(rows[1:4], ["X:~-95 Y:~+440", "A:~-12.25deg", "D:~450mm"])

    def test_rv1_does_not_invent_angle_from_default_zero(self):
        value = SimpleObservation("upper", 1, 10, "ok", (SimpleObject("green", 200, False),))
        rows = self.run_commands([*feed(encode(value, version=1)), "P", "S"])[0]
        self.assertEqual(rows[1:4], ["X:NA Y:NA", "A:NA", "D:~200mm"])

    def test_empty_and_fault_clear_previous_target(self):
        for status in ("empty", "fault"):
            snapshots = self.run_commands([*feed(packet()), "P", "S",
                *feed(packet(frame=2, objects=(), status=status), 110), "P", "S"])
            header = "UP NO GREEN" if status == "empty" else "UP FAULT"
            self.assertEqual(snapshots[1][:4], [header, "X:NA Y:NA", "A:NA", "D:NA"])

    def test_overflow_invalidates_then_recovers_at_newline(self):
        snapshots = self.run_commands([*feed(packet()), "P", *feed(b"x" * 2100, 110), "P", "S",
                                      *feed(b"\n" + packet(frame=2), 120), "P", "S"])
        self.assertEqual(snapshots[0][0], "UP FAULT")
        self.assertEqual(snapshots[1][:4], ["UP GREEN 1/1 mm", "X:-95 Y:+440", "A:-12.25deg", "D:450mm"])

    def test_receive_error_and_retry(self):
        snapshots = self.run_commands([*feed(packet()), "P", "E", "Q 1", "P", "S", "Q 0", "P",
                                      *feed(b"\n" + packet(frame=2), 120), "P", "S"])
        self.assertEqual(snapshots[0][0], "UP FAULT")
        self.assertEqual(snapshots[1][0], "UP GREEN 1/1 mm")

    def test_multitarget_rotation(self):
        objects = (SimpleObject("black"), SimpleObject("green", 100, False, False, 0.),
                   SimpleObject("orange"), SimpleObject("green", 200, False, False, 90.))
        rows = self.run_commands([*feed(packet(objects=objects), 2100), "P", "S"])[0]
        self.assertEqual(rows[:4], ["UP GREEN 2/2 mm", "X:~+200 Y:~+0", "A:~+90.00deg", "D:~200mm"])

    def test_non_green_objects_do_not_appear_on_oled(self):
        objects = (SimpleObject("black"), SimpleObject("orange"), SimpleObject("blue"))
        rows = self.run_commands([*feed(packet(objects=objects)), "P", "S"])[0]
        self.assertEqual(rows[:4], ["UP NO GREEN", "X:NA Y:NA", "A:NA", "D:NA"])

    def test_zero_distance_has_zero_xy_without_inventing_angle(self):
        objects = (SimpleObject("green", 0, False),)
        rows = self.run_commands([*feed(packet(objects=objects)), "P", "S"])[0]
        self.assertEqual(rows[1:4], ["X:~+0 Y:~+0", "A:NA", "D:~0mm"])


if __name__ == "__main__":
    unittest.main()
