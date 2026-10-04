"""Run the real task loops with a controlled tick source, without an MCU.

The CMSIS osDelayUntil implementation is extracted from the project's wrapper.
Only hardware IO, scheduler time advancement, and fatal halt are substituted.
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

PROJECT = Path(__file__).resolve().parents[2]


def extract_delay_until():
    path = PROJECT / 'Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c'
    text = path.read_text(encoding='utf-8-sig')
    match = re.search(r'osStatus_t osDelayUntil\s*\([^;]*?\)\s*\{', text)
    start = match.start()
    index = text.index('{', start) + 1
    depth = 1
    while depth:
        if text[index] == '{':
            depth += 1
        elif text[index] == '}':
            depth -= 1
        index += 1
    return text[start:index]


PRELUDE = r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <setjmp.h>
typedef uint32_t TickType_t;
typedef int osStatus_t;
enum {osOK=0, osErrorParameter=-4, osErrorISR=-6};
#define IS_IRQ() 0
#define osWaitForever UINT32_MAX
static uint32_t now, work_ticks, work_calls, waits, delay_race_ticks;
static uint32_t last_work_tick, spacing;
static osStatus_t forced_absolute_status, forced_relative_status;
static jmp_buf stopped;
static struct {float SpeedMeasure, PositionMeasure;} Motor_1;
static void fail(const char *message) {puts(message); longjmp(stopped,3);}
uint32_t xTaskGetTickCount(void) {return now;}
uint32_t osKernelGetTickCount(void) {
    if (forced_absolute_status != osOK) return now + 0U;
    return now;
}
void vTaskDelayUntil(TickType_t *base, TickType_t delay) {
    now=*base + delay; ++waits;
}
osStatus_t osDelay(uint32_t ticks) {
    if (ticks == 0U) fail("FAIL: zero delay does not yield");
    if (forced_relative_status != osOK) return forced_relative_status;
    now+=ticks; ++waits; return osOK;
}
void Error_Handler(void) {longjmp(stopped,2);}
static void work(void) {
    if (work_calls != 0U && waits < work_calls)
        fail("FAIL: periodic task spins without blocking");
    if (work_calls != 0U && now-last_work_tick != spacing)
        fail("FAIL: wrong periodic work spacing");
    if (work_calls == 3U) longjmp(stopped,1);
    last_work_tick=now; ++work_calls; now+=work_ticks;
}
void MotorControl_Step(void) {work();}
uint32_t UartService_GetReceiveErrorCount(void) {return 0;}
int UartService_SendString(const char *text) {(void)text; return 1;}
int UartService_SendTelemetry(float speed,float position) {
    (void)speed; (void)position; work(); return 1;
}
int UartService_StartReceive(void) {return 1;}
osStatus_t UartService_ReceiveCommand(uint8_t *command,uint32_t timeout) {
    (void)command; (void)timeout; return osErrorParameter;
}
typedef enum {
    MOTOR_CMD_NONE=0, MOTOR_CMD_SPEED, MOTOR_CMD_POSITION, MOTOR_CMD_STOP, MOTOR_CMD_INVALID
} MotorCommandKind_t;
typedef struct {MotorCommandKind_t kind; float value;} MotorControl_CommandResult_t;
MotorControl_CommandResult_t MotorControl_HandleCommandLine(const char *line) {
    (void)line;
    MotorControl_CommandResult_t result = {MOTOR_CMD_NONE, 0.0f};
    return result;
}
'''

TESTS = r'''
static int failures;
static void test(const char *name,void (*task)(void),uint32_t initial,
                 uint32_t cost,uint32_t interval,int expected,
                 osStatus_t absolute_error,osStatus_t relative_error,
                 uint32_t race_ticks) {
    now=initial; work_ticks=cost; spacing=interval;
    work_calls=0; waits=0; last_work_tick=0;
    forced_absolute_status=absolute_error;
    forced_relative_status=relative_error;
    delay_race_ticks=race_ticks;
    int result=setjmp(stopped);
    if (!result) task();
    if(result != expected) {
        printf("FAIL %s: expected %s, got %s\n",name,
               expected==1?"continued operation":"fatal handling",
               result==2?"Error_Handler":"unexpected execution");
        ++failures;
    } else printf("PASS %s\n",name);
}
int main(void) {
    test("motor normal cycle",AppTasks_MotorControl,100,0,1,1,osOK,osOK,0);
    test("motor reaches target tick",AppTasks_MotorControl,100,1,2,1,osOK,osOK,0);
    test("motor misses target tick",AppTasks_MotorControl,100,2,3,1,osOK,osOK,0);
    test("telemetry normal cycle",AppTasks_Telemetry,100,1,10,1,osOK,osOK,0);
    test("telemetry reaches target tick",AppTasks_Telemetry,100,10,20,1,osOK,osOK,0);
    test("telemetry misses target tick",AppTasks_Telemetry,100,11,21,1,osOK,osOK,0);
    test("motor normal tick wrap",AppTasks_MotorControl,UINT32_MAX,0,1,1,osOK,osOK,0);
    test("motor overrun across tick wrap",AppTasks_MotorControl,UINT32_MAX,1,2,1,osOK,osOK,0);
    test("deadline changes before delay reads tick",AppTasks_MotorControl,100,0,2,1,osOK,osOK,1);
    test("unexpected delay failure stays fatal",AppTasks_MotorControl,100,0,1,2,osErrorISR,osOK,0);
    test("recovery delay failure stays fatal",AppTasks_MotorControl,100,1,2,2,osOK,osErrorISR,0);
    printf("%d failed\n",failures);
    return failures?1:0;
}
'''


def main():
    compiler = shutil.which('gcc')
    if not compiler:
        raise SystemExit('Native gcc is required on PATH (not arm-none-eabi-gcc).')
    task_source = (PROJECT / 'App/Src/app_tasks.c').read_text(encoding='utf-8-sig')
    task_source = re.sub(r'^#include[^\n]*\n', '', task_source, flags=re.M)
    # Inject a tick-boundary race or a non-timing API failure at the OS boundary.
    wrapper = extract_delay_until().replace('  if (IS_IRQ()) {',
        '  now += delay_race_ticks;\n'
        '  if (forced_absolute_status != osOK) return forced_absolute_status;\n'
        '  if (IS_IRQ()) {', 1)
    with tempfile.TemporaryDirectory(prefix='rc-periodic-tests-') as directory:
        source = Path(directory) / 'periodic_tasks.c'
        binary = Path(directory) / 'periodic_tasks.exe'
        source.write_text(PRELUDE + '\n' + wrapper + '\n' + task_source + '\n' + TESTS,
                          encoding='utf-8')
        subprocess.run([compiler, '-std=c11', '-Wall', '-Wextra', '-Werror',
                        str(source), '-o', str(binary)], check=True)
        result = subprocess.run([str(binary)], check=False)
        raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
