#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "tm4c123gh6pm.h"
#include "TivaWare_C_Series-2.2.0.295/inc/hw_memmap.h"
#include "TivaWare_C_Series-2.2.0.295/inc/hw_types.h"
#include "TivaWare_C_Series-2.2.0.295/driverlib/sysctl.h"
#include "TivaWare_C_Series-2.2.0.295/driverlib/gpio.h"
#include "TivaWare_C_Series-2.2.0.295/driverlib/pin_map.h"
#include "TivaWare_C_Series-2.2.0.295/driverlib/i2c.h"
#include "TivaWare_C_Series-2.2.0.295/driverlib/timer.h"
#include "TivaWare_C_Series-2.2.0.295/driverlib/adc.h"

// Pin definitions
#define DOOR_PORT GPIO_PORTB_BASE
#define DOOR_PIN GPIO_PIN_0
#define DOOR_LOCK_PORT GPIO_PORTB_BASE
#define DOOR_LOCK_PIN GPIO_PIN_1
#define LOCK_TOGGLE_PORT GPIO_PORTB_BASE
#define LOCK_TOGGLE_PIN GPIO_PIN_5
#define DEBOUNCE_DELAY_MS 50

#define ADC_TIMEOUT_TICKS   1000
#define MIN_SPEED           0.0f
#define MAX_SPEED           30.0f
#define SPEED_THRESHOLD     10.0f

#define TRIG_PORT GPIO_PORTD_BASE
#define TRIG_PIN GPIO_PIN_0
#define ECHO_PORT GPIO_PORTD_BASE
#define ECHO_PIN GPIO_PIN_1

#define SWITCH_PORT GPIO_PORTE_BASE
#define SWITCH_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3)
#define S_KEY_PIN GPIO_PIN_3

#define LED_PORT GPIO_PORTA_BASE
#define GREEN_LED GPIO_PIN_2
#define YELLOW_LED GPIO_PIN_3
#define RED_LED GPIO_PIN_4

#define BUZZER_PORT GPIO_PORTF_BASE
#define BUZZER_PIN GPIO_PIN_2

#define I2C_LCD_ADDRESS 0x27
#define LCD_COLS 16
#define LCD_ROWS 2
#define LCD_CLEAR 0x01
#define LCD_HOME 0x02
#define LCD_ENTRY_MODE 0x06
#define LCD_DISPLAY_ON 0x0C
#define LCD_SET_DDRAM 0x80
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

// Data structures
typedef struct {
    float speed;
} SpeedMessage_t;

typedef struct {
    bool door_open;
    bool doors_locked;
} DoorState_t;

typedef struct {
    uint8_t command; // 0: clear, 1: regular update, 2: high-priority message
    uint8_t row;
    uint8_t col;
    uint8_t duration; // For command 2: 0 for 2s, 1 for 10s
    char text[LCD_COLS + 1];
    TaskHandle_t sender; // To notify sender when update is complete
} LCDMessage_t;

typedef struct {
    bool d_pressed;
    bool p_pressed;
    bool r_pressed;
    bool ignition_off;
} SwitchState_t;

typedef enum {
    STATUS_OK = 0,
    STATUS_FAIL = -1,
    STATUS_TIMEOUT = -2
} SystemStatus_t;

typedef struct {
    bool is_initialized;
    SystemStatus_t last_error;
    uint32_t error_count;
} SystemState_t;

// LCD display buffer
typedef struct {
    char buffer[LCD_ROWS][LCD_COLS + 1]; // Current content of each row
    TickType_t last_update[LCD_ROWS]; // Last update time for each row
    bool locked; // True when high-priority message is active
    TickType_t lock_until; // Time until lock expires
    bool was_high_priority[LCD_ROWS]; // Track if row had high-priority message
} LCDBuffer_t;

// Global variables
static QueueHandle_t lcdQueue;
static QueueHandle_t distanceQueue;
static QueueHandle_t stateQueue;
static QueueHandle_t speedQueue;
static SemaphoreHandle_t lcdSemaphore;
static SemaphoreHandle_t switchStateSemaphore;
static SemaphoreHandle_t doorStateSemaphore;
static SemaphoreHandle_t RstateSemaphore;
static SemaphoreHandle_t xSpeedMutex;
static SemaphoreHandle_t lcdQueueMutex;
static TaskHandle_t sensorTaskHandle = NULL;
static TaskHandle_t buzzerTaskHandle = NULL;
static DoorState_t doorState = {true, false};
static SwitchState_t switchState = {false, false, false, true};
static SystemState_t systemState = {false, STATUS_OK, 0};
static float g_vehicleSpeed = 0.0f;
static bool g_forceZeroSpeed = false;
static bool g_doorsLocked = false;
static bool g_manualLockOverride = false;
static bool g_manualLockAllowed = false;
static bool g_wasAboveThreshold = false;
static uint32_t g_zeroSpeedTicks = 0;
static bool g_doorOpenAlert = false;
static bool g_systemStopped = false;
static bool g_gearBlocked = false;
static bool g_closeDoorFirstDisplayed = false;
static bool g_sensorResumed = false;
static LCDBuffer_t lcdBuffer = {{{0}}, {0}, false, 0, {false}};
static bool last_p_lock_state = false; // Track P's lock state
#define ZERO_SPEED_RESET_TICKS 5
#define LCD_MIN_UPDATE_INTERVAL pdMS_TO_TICKS(50)
static bool saraFlag = false;

// Initialize I2C0 (PB2-SCL, PB3-SDA)
void InitI2C(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0) || !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    GPIO_PORTB_AFSEL_R |= (GPIO_PIN_2 | GPIO_PIN_3);
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & ~0x0000FF00) | (3 << 8) | (3 << 12);
    GPIO_PORTB_DEN_R |= (GPIO_PIN_2 | GPIO_PIN_3);
    GPIO_PORTB_ODR_R |= GPIO_PIN_3;
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
}

// Initialize ADC for potentiometer on PE5
SystemStatus_t InitADC(void) {
    if (systemState.is_initialized) return STATUS_OK;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    if (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0) || 
        !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) {
        systemState.last_error = STATUS_FAIL;
        systemState.error_count++;
        return STATUS_FAIL;
    }
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH8 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
    systemState.is_initialized = true;
    return STATUS_OK;
}

// Send byte to I2C LCD
void LCDWriteByte(uint8_t data, uint8_t rs) {
    uint8_t data_h = (data & 0xF0);
    uint8_t data_l = ((data << 4) & 0xF0);
    uint8_t backlight = LCD_BACKLIGHT;

    I2CMasterSlaveAddrSet(I2C0_BASE, I2C_LCD_ADDRESS, false);
    I2CMasterDataPut(I2C0_BASE, data_h | rs | backlight | 0x04);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(I2C0_BASE));
    I2CMasterDataPut(I2C0_BASE, data_h | rs | backlight);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(I2C0_BASE));

    I2CMasterDataPut(I2C0_BASE, data_l | rs | backlight | 0x04);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(I2C0_BASE));
    I2CMasterDataPut(I2C0_BASE, data_l | rs | backlight);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(I2C0_BASE));
}

// Initialize LCD
void InitLCD(void) {
    xSemaphoreTake(lcdSemaphore, portMAX_DELAY);
    SysCtlDelay(1600000);
    LCDWriteByte(0x03, 0); SysCtlDelay(200000);
    LCDWriteByte(0x03, 0); SysCtlDelay(40000);
    LCDWriteByte(0x03, 0);
    LCDWriteByte(0x02, 0);
    LCDWriteByte(0x28, 0);
    LCDWriteByte(LCD_DISPLAY_ON, 0);
    LCDWriteByte(LCD_CLEAR, 0); SysCtlDelay(80000);
    LCDWriteByte(LCD_ENTRY_MODE, 0);
    xSemaphoreGive(lcdSemaphore);

    // Initialize LCD buffer
    for (int i = 0; i < LCD_ROWS; i++) {
        memset(lcdBuffer.buffer[i], ' ', LCD_COLS);
        lcdBuffer.buffer[i][LCD_COLS] = '\0';
        lcdBuffer.last_update[i] = 0;
        lcdBuffer.was_high_priority[i] = false;
    }
}

// LCD print string
void LCDPrint(char *str, uint8_t row, uint8_t col) {
    xSemaphoreTake(lcdSemaphore, portMAX_DELAY);
    uint8_t addr = (row == 0) ? (0x00 + col) : (0x40 + col);
    LCDWriteByte(LCD_SET_DDRAM | addr, 0);
    while (*str && col < LCD_COLS) {
        LCDWriteByte(*str++, 1);
        col++;
    }
    xSemaphoreGive(lcdSemaphore);
}

// Clear LCD (fully reset buffer and states)
void LCDClear(void) {
    xSemaphoreTake(lcdSemaphore, portMAX_DELAY);
    LCDWriteByte(LCD_CLEAR, 0);
    SysCtlDelay(80000);
    xSemaphoreGive(lcdSemaphore);

    // Fully clear LCD buffer and reset states
    for (int i = 0; i < LCD_ROWS; i++) {
        memset(lcdBuffer.buffer[i], ' ', LCD_COLS);
        lcdBuffer.buffer[i][LCD_COLS] = '\0';
        lcdBuffer.last_update[i] = xTaskGetTickCount();
        lcdBuffer.was_high_priority[i] = false;
    }
    lcdBuffer.locked = false;
    lcdBuffer.lock_until = 0;
    g_closeDoorFirstDisplayed = false; // Reset on clear
}

// Update LCD buffer and display if changed
bool UpdateLCDBuffer(uint8_t row, uint8_t col, const char *text, bool force, bool is_high_priority) {
    if (row >= LCD_ROWS || col >= LCD_COLS) return false;

    // Prepare new text
    char new_text[LCD_COLS + 1];
    strncpy(new_text, text, LCD_COLS);
    new_text[LCD_COLS] = '\0';
    size_t text_len = strlen(new_text);
    for (size_t i = text_len; i < LCD_COLS - col; i++) {
        new_text[i] = ' ';
    }

    TickType_t current_time = xTaskGetTickCount();

    // Clear entire row if force, high-priority transition, or col == 0
    if (force || lcdBuffer.was_high_priority[row] || col == 0) {
        memset(lcdBuffer.buffer[row], ' ', LCD_COLS);
        lcdBuffer.buffer[row][LCD_COLS] = '\0';
    }

    // Check if update is needed
    if (!force && strcmp(lcdBuffer.buffer[row] + col, new_text) == 0 &&
        (current_time - lcdBuffer.last_update[row] < LCD_MIN_UPDATE_INTERVAL)) {
        lcdBuffer.was_high_priority[row] = is_high_priority;
        return false;
    }

    // Update buffer
    strncpy(lcdBuffer.buffer[row] + col, new_text, LCD_COLS - col);
    lcdBuffer.last_update[row] = current_time;
    lcdBuffer.was_high_priority[row] = is_high_priority;

    // Update LCD
    LCDPrint(new_text, row, col);
    return true;
}

// Read speed from potentiometer
float ReadSpeedFromPotentiometer(void) {
    if (!systemState.is_initialized) {
        if (InitADC() != STATUS_OK) {
            return MIN_SPEED;
        }
    }
    uint32_t timeout = ADC_TIMEOUT_TICKS;
    uint32_t adc_value = 0;
    ADCProcessorTrigger(ADC0_BASE, 3);
    while (!ADCIntStatus(ADC0_BASE, 3, false) && timeout--) {
        if (timeout == 0) {
            systemState.last_error = STATUS_TIMEOUT;
            systemState.error_count++;
            return MIN_SPEED;
        }
    }
    ADCIntClear(ADC0_BASE, 3);
    if (ADCSequenceDataGet(ADC0_BASE, 3, &adc_value) != 0) {
        float speed = ((float)adc_value / 4095.0f) * MAX_SPEED;
        if (speed < MIN_SPEED) return MIN_SPEED;
        if (speed > MAX_SPEED) return MAX_SPEED;
        return speed;
    }
    systemState.last_error = STATUS_FAIL;
    systemState.error_count++;
    return MIN_SPEED;
}

// Initialize door and lock pins
void InitDoorLock(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    GPIOPinTypeGPIOInput(DOOR_PORT, DOOR_PIN | LOCK_TOGGLE_PIN);
    GPIOPadConfigSet(DOOR_PORT, DOOR_PIN | LOCK_TOGGLE_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPinTypeGPIOOutput(DOOR_LOCK_PORT, DOOR_LOCK_PIN);
    GPIOPinWrite(DOOR_LOCK_PORT, DOOR_LOCK_PIN, 0);
}

// Initialize HC-SR04 and LEDs
void InitSensor(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    GPIOPinTypeGPIOOutput(TRIG_PORT, TRIG_PIN);
    GPIOPinTypeGPIOInput(ECHO_PORT, ECHO_PIN);
    GPIOPinTypeGPIOOutput(LED_PORT, GREEN_LED | YELLOW_LED | RED_LED);
    GPIOPinWrite(LED_PORT, GREEN_LED | YELLOW_LED | RED_LED, 0);

    TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 0xFFFFFFFF);
}

// Initialize switches
void InitSwitches(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOInput(SWITCH_PORT, SWITCH_PINS);
    GPIOPadConfigSet(SWITCH_PORT, SWITCH_PINS, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

// Initialize buzzer
void InitBuzzer(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(BUZZER_PORT, BUZZER_PIN);
    GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, 0);
}

// Measure distance
float GetDistance(void) {
    GPIOPinWrite(TRIG_PORT, TRIG_PIN, 0);
    SysCtlDelay(160);
    GPIOPinWrite(TRIG_PORT, TRIG_PIN, TRIG_PIN);
    SysCtlDelay(400);
    GPIOPinWrite(TRIG_PORT, TRIG_PIN, 0);

    uint32_t timeout = 2000000;
    while (GPIOPinRead(ECHO_PORT, ECHO_PIN) == 0) {
        if (--timeout == 0) return -1.0f;
    }

    TimerLoadSet(TIMER0_BASE, TIMER_A, 0xFFFFFFFF);
    TimerEnable(TIMER0_BASE, TIMER_A);

    timeout = 2000000;
    while (GPIOPinRead(ECHO_PORT, ECHO_PIN) != 0) {
        if (--timeout == 0) {
            TimerDisable(TIMER0_BASE, TIMER_A);
            return -1.0f;
        }
    }

    TimerDisable(TIMER0_BASE, TIMER_A);
    uint32_t time = 0xFFFFFFFF - TimerValueGet(TIMER0_BASE, TIMER_A);
    float distance = ((float)time * 34300.0f) / (2.0f * 80000000.0f);
    return distance;
}

// Send LCD message with completion notification
bool SendLCDMessage(LCDMessage_t *msg, TickType_t timeout) {
    if (xSemaphoreTake(lcdQueueMutex, timeout) == pdTRUE) {
        msg->sender = xTaskGetCurrentTaskHandle();
        if (xQueueSend(lcdQueue, msg, timeout) == pdPASS) {
            ulTaskNotifyTake(pdTRUE, timeout);
            xSemaphoreGive(lcdQueueMutex);
            return true;
        }
        xSemaphoreGive(lcdQueueMutex);
        return false;
    }
    return false;
}

// LCD Task
void LCDTask(void *pvParameters) {
    LCDMessage_t msg;
    while (1) {
        if (xQueueReceive(lcdQueue, &msg, portMAX_DELAY) == pdPASS) {
            TickType_t current_time = xTaskGetTickCount();

            // Check if LCD is locked by a high-priority message
            if (lcdBuffer.locked && current_time < lcdBuffer.lock_until) {
                if (msg.command == 2) {
                    lcdBuffer.locked = false; // Allow new high-priority message
                } else {
                    if (msg.sender) xTaskNotifyGive(msg.sender);
                    continue;
                }
            }

            if (msg.command == 0) {
                LCDClear();
                lcdBuffer.locked = false;
                g_closeDoorFirstDisplayed = false; // Reset on clear
            } else if (msg.command == 1) {
                // Clear row if it was high-priority
                if (lcdBuffer.was_high_priority[msg.row]) {
                    UpdateLCDBuffer(msg.row, 0, "", true, false);
                }
                UpdateLCDBuffer(msg.row, msg.col, msg.text, false, false);
            } else if (msg.command == 2) {
                UpdateLCDBuffer(msg.row, msg.col, msg.text, true, true);
                lcdBuffer.locked = true;
                lcdBuffer.lock_until = current_time + pdMS_TO_TICKS(msg.duration == 1 ? 10000 : 2000);
            }

            if (msg.sender) xTaskNotifyGive(msg.sender);
        }
    }
}

// Potentiometer Task
void PotentiometerTask(void *pvParameters) {
    LCDMessage_t msg;
    if (InitADC() != STATUS_OK) {
        msg.command = 1;
        msg.row = 1;
        msg.col = 0;
        snprintf(msg.text, LCD_COLS + 1, "ADC Init Fail");
        SendLCDMessage(&msg, pdMS_TO_TICKS(10));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    while (1) {
        bool force_zero;
        float speed = MIN_SPEED;
        if (xSemaphoreTake(switchStateSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
            bool ignition_on = !switchState.ignition_off;
            bool gear_in_d = switchState.d_pressed;
            bool gear_in_r = switchState.r_pressed;
            xSemaphoreGive(switchStateSemaphore);

            xSemaphoreTake(doorStateSemaphore, pdMS_TO_TICKS(10));
            bool door_open = doorState.door_open;
            xSemaphoreGive(doorStateSemaphore);

            // Check current speed
            float current_speed;
            xSemaphoreTake(xSpeedMutex, pdMS_TO_TICKS(10));
            current_speed = g_vehicleSpeed;
            xSemaphoreGive(xSpeedMutex);

            // Handle door open in D or R
            if (door_open && (gear_in_d || gear_in_r)) {
                xSemaphoreTake(xSpeedMutex, pdMS_TO_TICKS(10));
                g_vehicleSpeed = 0.0f;
                g_gearBlocked = true;
                g_forceZeroSpeed = true;
                xSemaphoreGive(xSpeedMutex);

                if (!g_closeDoorFirstDisplayed) {
                    msg.command = 2;
                    msg.duration = 1; // 10s
                    msg.row = 0;
                    msg.col = 0;
                    snprintf(msg.text, LCD_COLS + 1, "Close Door First");
                    SendLCDMessage(&msg, pdMS_TO_TICKS(10));
                    msg.command = 2;
                    msg.row = 1;
                    msg.col = 0;
                    snprintf(msg.text, LCD_COLS + 1, "S:0.0 km/h");
                    SendLCDMessage(&msg, pdMS_TO_TICKS(10));
                    g_closeDoorFirstDisplayed = true;
                }
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            } else if (!door_open && g_closeDoorFirstDisplayed && (gear_in_d || gear_in_r)) {
                // Door closed, reset gear block and clear message
                xSemaphoreTake(xSpeedMutex, pdMS_TO_TICKS(10));
                g_gearBlocked = false;
                g_forceZeroSpeed = false;
                g_closeDoorFirstDisplayed = false;
                xSemaphoreGive(xSpeedMutex);

                msg.command = 0;
                SendLCDMessage(&msg, pdMS_TO_TICKS(10));

                msg.command = 1;
                msg.row = 0;
                msg.col = 0;
                snprintf(msg.text, LCD_COLS + 1, gear_in_d ? "Key: D" : "Key: R");
                SendLCDMessage(&msg, pdMS_TO_TICKS(10));
            }

            if (ignition_on && (gear_in_d || gear_in_r) && !g_gearBlocked && !g_systemStopped) {
                xSemaphoreTake(doorStateSemaphore, pdMS_TO_TICKS(10));
                door_open = doorState.door_open;
                xSemaphoreGive(doorStateSemaphore);
                if (xSemaphoreTake(xSpeedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    force_zero = g_forceZeroSpeed;
                    if (!force_zero) {
                        speed = ReadSpeedFromPotentiometer();
                        if (systemState.last_error == STATUS_FAIL || systemState.last_error == STATUS_TIMEOUT) {
                            msg.command = 1;
                            msg.row = 1;
                            msg.col = 0;
                            snprintf(msg.text, 8, "ADC Err:%d", systemState.last_error);
                            SendLCDMessage(&msg, pdMS_TO_TICKS(10));
                        } else {
                            g_vehicleSpeed = speed;
                            xSemaphoreTake(doorStateSemaphore, pdMS_TO_TICKS(10));
                            bool isAboveThreshold = (speed > SPEED_THRESHOLD);
                            if (isAboveThreshold != g_wasAboveThreshold) {
                                g_manualLockOverride = false;
                                g_manualLockAllowed = false;
                                g_wasAboveThreshold = isAboveThreshold;
                            }
                            if (!g_manualLockOverride) {
                                if (speed > SPEED_THRESHOLD && !g_doorsLocked && !doorState.door_open) {
                                    GPIOPinWrite(DOOR_LOCK_PORT, DOOR_LOCK_PIN, DOOR_LOCK_PIN);
                                    g_doorsLocked = true;
                                    doorState.doors_locked = true;
                                    g_manualLockAllowed = false;
                                } else if (speed <= SPEED_THRESHOLD && g_doorsLocked) {
                                    GPIOPinWrite(DOOR_LOCK_PORT, DOOR_LOCK_PIN, 0);
                                    g_doorsLocked = false;
                                    doorState.doors_locked = false;
                                    g_manualLockAllowed = true;
                                }
                            }
                            if (!g_systemStopped) {
                                msg.command = 1;
                                msg.row = 0;
                                msg.col = 8;
                                snprintf(msg.text, 8, "S:%.1f", speed);
                                SendLCDMessage(&msg, pdMS_TO_TICKS(10));
                            }
                            if (speed == MIN_SPEED) {
                                g_zeroSpeedTicks++;
                                if (g_zeroSpeedTicks >= ZERO_SPEED_RESET_TICKS) {
                                    g_manualLockOverride = false;
                                    g_zeroSpeedTicks = 0;
                                }
                            } else {
                                g_zeroSpeedTicks = 0;
                            }
                            xSemaphoreGive(doorStateSemaphore);
                        }
                    }
                    xSemaphoreGive(xSpeedMutex);
                }
            } else {
                if (xSemaphoreTake(switchStateSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
                    if (switchState.ignition_off && g_doorsLocked && !g_manualLockAllowed) {
                        xSemaphoreTake(doorStateSemaphore, pdMS_TO_TICKS(10));
                        GPIOPinWrite(DOOR_LOCK_PORT, DOOR_LOCK_PIN, 0);
                        g_doorsLocked = false;
                        doorState.doors_locked = false;
                        g_manualLockOverride = false;
                        g_manualLockAllowed = true;
                        g_wasAboveThreshold = false;
                        g_closeDoorFirstDisplayed = false; // Reset on ignition off
                        xSemaphoreGive(doorStateSemaphore);
                    }
                    if (!gear_in_d && !gear_in_r && !g_systemStopped) {
                        msg.command = 1;
                        msg.row = 0;
                        msg.col = 8;
                        snprintf(msg.text, 8, "       ");
                        SendLCDMessage(&msg, pdMS_TO_TICKS(10));
                        g_manualLockOverride = false;
                        g_zeroSpeedTicks = 0;
                        g_wasAboveThreshold = false;
                        g_closeDoorFirstDisplayed = false; // Reset when not in D or R
                    }
                    xSemaphoreGive(switchStateSemaphore);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Door and Lock Task
void DoorLockTask(void *pvParameters) {
    bool last_door_state = true;
    bool last_toggle_state = true;

    while (1) {
        uint8_t pb = GPIOPinRead(DOOR_PORT, DOOR_PIN | LOCK_TOGGLE_PIN);
        bool door_open = (pb & DOOR_PIN) == 0;
        bool toggle_pressed = (pb & LOCK_TOGGLE_PIN) == 0;

        xSemaphoreTake(doorStateSemaphore, pdMS_TO_TICKS(10));
        doorState.door_open = door_open;

        if (!toggle_pressed && last_toggle_state) {
            if (xSemaphoreTake(xSpeedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_doorsLocked = !g_doorsLocked;
                doorState.doors_locked = g_doorsLocked;
                g_manualLockOverride = true;
                if (g_manualLockAllowed) {
                    g_manualLockOverride = true;
                }
                GPIOPinWrite(DOOR_LOCK_PORT, DOOR_LOCK_PIN, doorState.doors_locked ? DOOR_LOCK_PIN : 0);
                xSemaphoreGive(xSpeedMutex);
            }
        }

        xSemaphoreTake(switchStateSemaphore, pdMS_TO_TICKS(10));
        bool ignition_off = switchState.ignition_off;
        xSemaphoreGive(switchStateSemaphore);

        if (ignition_off && g_doorsLocked && !g_manualLockAllowed) {
            xSemaphoreTake(xSpeedMutex, pdMS_TO_TICKS(10));
            g_doorsLocked = false;
            doorState.doors_locked = false;
            g_manualLockOverride = false;
            g_manualLockAllowed = true;
            GPIOPinWrite(DOOR_LOCK_PORT, DOOR_LOCK_PIN, 0);
            g_closeDoorFirstDisplayed = false; // Reset on ignition off
            xSemaphoreGive(xSpeedMutex);
        }

        // Always update door/lock status
        LCDMessage_t msg;
        msg.command = 1;
        msg.row = 1;
        msg.col = 8;
        snprintf(msg.text, LCD_COLS + 1, "D:%s L:%s", doorState.door_open ? "O" : "C", doorState.doors_locked ? "L" : "U");
        SendLCDMessage(&msg, pdMS_TO_TICKS(10));

        xSemaphoreGive(doorStateSemaphore);

        last_door_state = door_open;
        last_toggle_state = toggle_pressed;

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Sensor Task
void SensorTask(void *pvParameters) {
    LCDMessage_t msg;
    InitSensor();

    while (1) {
        xSemaphoreTake(RstateSemaphore, portMAX_DELAY);
        xSemaphoreTake(switchStateSemaphore, pdMS_TO_TICKS(10));
        bool d_pressed = switchState.d_pressed;
        bool r_pressed = switchState.r_pressed;
        bool ignition_off = switchState.ignition_off;
        xSemaphoreGive(switchStateSemaphore);

        bool door_open;
        xSemaphoreTake(doorStateSemaphore, pdMS_TO_TICKS(10));
        door_open = doorState.door_open;
        xSemaphoreGive(doorStateSemaphore);

        // Skip if not in R gear, gear blocked, system stopped, or door open
        if (g_gearBlocked || !r_pressed || g_systemStopped || door_open) {
            g_sensorResumed = false;
            xSemaphoreGive(RstateSemaphore);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        float distance = GetDistance();

        if (g_sensorResumed) {
            if (distance <= 2.5f && distance > 0.0f) {
                xSemaphoreTake(xSpeedMutex, pdMS_TO_TICKS(10));
                g_vehicleSpeed = 0.0f;
                g_forceZeroSpeed = true;
                g_systemStopped = true;
                xSemaphoreGive(xSpeedMutex);

                // Display "Car Stopped" on row 0, col 0
                msg.command = 2;
                msg.duration = 1; // 10s
                msg.row = 0;
                msg.col = 0;
                snprintf(msg.text, LCD_COLS + 1, "Car Stopped");
                SendLCDMessage(&msg, pdMS_TO_TICKS(10));

                // Display "S:0.0 km/h" on row 1, col 0
                msg.command = 2;
                msg.duration = 1;
                msg.row = 1;
                msg.col = 0;
                snprintf(msg.text, LCD_COLS + 1, "S:0.0 km/h");
                SendLCDMessage(&msg, pdMS_TO_TICKS(10));

                // Keep LEDs red and buzzer fast
                GPIOPinWrite(LED_PORT, GREEN_LED | YELLOW_LED | RED_LED, RED_LED);
                vTaskResume(buzzerTaskHandle);
            } else if (!g_systemStopped) {
                msg.command = 1;
                msg.row = 1;
                msg.col = 0;

                if (distance > 400) {
                    strcpy(msg.text, "Too Far");
                } else if (distance > 2.6) {
                    snprintf(msg.text, LCD_COLS + 1, "%.1f cm", distance);
                } else {
                    strcpy(msg.text, "No Echo");
                }

                xQueueSend(distanceQueue, &distance, 0);
                SendLCDMessage(&msg, pdMS_TO_TICKS(10));

                if (distance > 100.0f) {
                    GPIOPinWrite(LED_PORT, GREEN_LED | YELLOW_LED | RED_LED, GREEN_LED);
                    vTaskSuspend(buzzerTaskHandle);
                } else if (distance < 30.0f && distance > 0.0f) {
                    GPIOPinWrite(LED_PORT, GREEN_LED | YELLOW_LED | RED_LED, RED_LED);
                    vTaskResume(buzzerTaskHandle);
                } else if (distance >= 30.0f && distance <= 100.0f) {
                    GPIOPinWrite(LED_PORT, GREEN_LED | YELLOW_LED | RED_LED, YELLOW_LED);
                    vTaskResume(buzzerTaskHandle);
                } else {
                    GPIOPinWrite(LED_PORT, GREEN_LED | YELLOW_LED | RED_LED, 0);
                    vTaskSuspend(buzzerTaskHandle);
                }
            }
        } else {
            g_sensorResumed = true; // Skip first update
        }

        xSemaphoreGive(RstateSemaphore);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Buzzer Task
void BuzzerTask(void *pvParameters) {
    float d = 10000;
    while (1) {
        if (xQueueReceive(distanceQueue, &d, portMAX_DELAY) == pdPASS) {
            if (d >= 30.0f && d <= 100.0f) {
                GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, BUZZER_PIN);
                vTaskDelay(pdMS_TO_TICKS(70));
                GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(70));
            } else if (d < 30.0f && d > 0.0f) {
                GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, BUZZER_PIN);
                vTaskDelay(pdMS_TO_TICKS(20));
                GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(20));
            } else {
                GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        GPIOPinWrite(BUZZER_PORT, BUZZER_PIN, 0);
    }
}

// Ignition Switch Task
void IgnitionSwitchTask(void *pvParameters) {
    InitSwitches();
    bool last_ignition_state = true;

    while (1) {
        bool ignition_off = (GPIOPinRead(SWITCH_PORT, GPIO_PIN_3) & GPIO_PIN_3) != 0;

        xSemaphoreTake(switchStateSemaphore, portMAX_DELAY);
        switchState.ignition_off = ignition_off;

        if (ignition_off != last_ignition_state) {
            LCDMessage_t msg;
            if (g_systemStopped) {
                xSemaphoreGive(switchStateSemaphore);
                last_ignition_state = ignition_off;
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
            msg.command = 0; // Clear LCD
            SendLCDMessage(&msg, pdMS_TO_TICKS(10));

            msg.command = 1;
            msg.row = 0;
            msg.col = 0;

            if (ignition_off) {
                // Ignition turned off
                xSemaphoreTake(xSpeedMutex, pdMS_TO_TICKS(10));
                xSemaphoreTake(doorStateSemaphore, pdMS_TO_TICKS(10));
                g_doorsLocked = false;
                doorState.doors_locked = false;
                g_manualLockOverride = false;
                g_manualLockAllowed = true;
                g_closeDoorFirstDisplayed = false; // Reset on ignition off
                GPIOPinWrite(DOOR_LOCK_PORT, DOOR_LOCK_PIN, 0);
                xSemaphoreGive(doorStateSemaphore);
                xSemaphoreGive(xSpeedMutex);

                snprintf(msg.text, LCD_COLS + 1, "ignition is off");
                SendLCDMessage(&msg, pdMS_TO_TICKS(10));
                saraFlag = false;
                g_gearBlocked = false;
                if (sensorTaskHandle != NULL) {
                    vTaskSuspend(sensorTaskHandle);
                }
            } else if (saraFlag) {
                msg.command = 2;
                msg.duration = 0;
                snprintf(msg.text, LCD_COLS + 1, "close engine");
                SendLCDMessage(&msg, pdMS_TO_TICKS(10));
                if (sensorTaskHandle != NULL) {
                    vTaskSuspend(sensorTaskHandle);
                }
            } else if (!ignition_off && !switchState.p_pressed && !switchState.d_pressed && !switchState.r_pressed) {
                saraFlag = true;
                if (sensorTaskHandle != NULL) {
                    vTaskSuspend(sensorTaskHandle);
                }
            } else if (!ignition_off && !switchState.p_pressed && (switchState.d_pressed || switchState.r_pressed)) {
                msg.command = 2;
                msg.duration = 0;
                snprintf(msg.text, LCD_COLS + 1, "close engine");
                saraFlag = true;
                SendLCDMessage(&msg, pdMS_TO_TICKS(10));
                if (sensorTaskHandle != NULL) {
                    vTaskSuspend(sensorTaskHandle);
                }
            } else {
                // Ignition turned on, check if P is pressed
                if (switchState.p_pressed && !switchState.d_pressed && !switchState.r_pressed) {
                    // P is pressed, proceed normally
                    snprintf(msg.text, LCD_COLS + 1, "Key: P");
                    SendLCDMessage(&msg, pdMS_TO_TICKS(10));
                } else {
                    // P is not pressed (D, R, or no switch), treat as ignition off
                    snprintf(msg.text, LCD_COLS + 1, "ignition is off");
                    SendLCDMessage(&msg, pdMS_TO_TICKS(10));
                    switchState.ignition_off = true; // Force ignition off state
                    if (sensorTaskHandle != NULL) {
                        vTaskSuspend(sensorTaskHandle);
                    }
                }
            }
        }
        xSemaphoreGive(switchStateSemaphore);
        last_ignition_state = ignition_off;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Switch Task
void SwitchTask(void *pvParameters) {
    InitSwitches();
    bool last_d_state = false;
    bool last_p_state = false;
    bool last_r_state = false;
    bool last_door_state = true;

    while (1) {
        uint8_t pe = GPIOPinRead(SWITCH_PORT, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
        bool d_switch_on = (pe & GPIO_PIN_0) == 0;
        bool p_switch_on = (pe & GPIO_PIN_1) == 0;
        bool r_switch_on = (pe & GPIO_PIN_2) == 0;

        xSemaphoreTake(switchStateSemaphore, portMAX_DELAY);
        switchState.d_pressed = d_switch_on;
        switchState.p_pressed = p_switch_on;
        switchState.r_pressed = r_switch_on;

        bool door_open;
        xSemaphoreTake(doorStateSemaphore, pdMS_TO_TICKS(10));
        door_open = doorState.door_open;
        xSemaphoreGive(doorStateSemaphore);

        LCDMessage_t msg;
        msg.command = 1;
        msg.row = 0;
        msg.col = 0;

        // Handle door open with D or R gear
        if ((d_switch_on || r_switch_on) && door_open && !g_gearBlocked) {
            xSemaphoreTake(xSpeedMutex, pdMS_TO_TICKS(10));
            g_vehicleSpeed = 0.0f;
            g_gearBlocked = true;
            g_forceZeroSpeed = true;
            g_systemStopped = false;
            xSemaphoreGive(xSpeedMutex);

            vTaskSuspend(sensorTaskHandle);
            xSemaphoreTake(RstateSemaphore, pdMS_TO_TICKS(10));

            if (!g_closeDoorFirstDisplayed) {
                msg.command = 2;
                msg.duration = 1;
                msg.row = 0;
                msg.col = 0;
                snprintf(msg.text, LCD_COLS + 1, "Close Door First");
                SendLCDMessage(&msg, pdMS_TO_TICKS(10));
                msg.command = 2;
                msg.row = 1;
                msg.col = 0;
                snprintf(msg.text, LCD_COLS + 1, "S:0.0 km/h");
                SendLCDMessage(&msg, pdMS_TO_TICKS(10));
                g_closeDoorFirstDisplayed = true;
            }
            g_doorOpenAlert = false;
            vTaskSuspend(buzzerTaskHandle);
        }
        // Handle door closed with D or R gear
        else if (g_gearBlocked && !door_open && (d_switch_on || r_switch_on)) {
            xSemaphoreTake(xSpeedMutex, pdMS_TO_TICKS(10));
            g_gearBlocked = false;
            g_forceZeroSpeed = false;
            g_closeDoorFirstDisplayed = false;
            g_vehicleSpeed = 0.0f; // Reset speed
            g_systemStopped = false;
            xSemaphoreGive(xSpeedMutex);

            msg.command = 0;
            SendLCDMessage(&msg, pdMS_TO_TICKS(10));

            msg.command = 1;
            msg.row = 0;
            msg.col = 0;
            if (d_switch_on) {
                snprintf(msg.text, LCD_COLS + 1, "Key: D");
            } else if (r_switch_on) {
                snprintf(msg.text, LCD_COLS + 1, "Key: R");
            }
            SendLCDMessage(&msg, pdMS_TO_TICKS(10));

            if (r_switch_on) {
                vTaskResume(sensorTaskHandle);
                xSemaphoreGive(RstateSemaphore);
            } else {
                // Clear distance display when not in R
                msg.command = 1;
                msg.row = 1;
                msg.col = 0;
                snprintf(msg.text, LCD_COLS + 1, "           ");
                SendLCDMessage(&msg, pdMS_TO_TICKS(10));
            }
            g_doorOpenAlert = false;
            vTaskSuspend(buzzerTaskHandle);
        }
        // Handle gear changes when ignition is on
        else if ((d_switch_on != last_d_state || p_switch_on != last_p_state || r_switch_on != last_r_state) && !saraFlag && !switchState.ignition_off) {
            int pressed_count = (d_switch_on ? 1 : 0) + (p_switch_on ? 1 : 0) + (r_switch_on ? 1 : 0);
            GPIOPinWrite(LED_PORT, GREEN_LED | YELLOW_LED | RED_LED, 0);

            // Clear system stopped state when switching to P or D
            if ((d_switch_on || p_switch_on) && !r_switch_on && g_systemStopped) {
                xSemaphoreTake(xSpeedMutex, pdMS_TO_TICKS(10));
                g_forceZeroSpeed = false;
                g_systemStopped = false;
                g_vehicleSpeed = 0.0f;
                g_closeDoorFirstDisplayed = false;
                xSemaphoreGive(xSpeedMutex);

                msg.command = 0;
                SendLCDMessage(&msg, pdMS_TO_TICKS(10));
            }

            // Update lock state for gear changes
            if (!g_manualLockOverride) {
                xSemaphoreTake(xSpeedMutex, pdMS_TO_TICKS(10));
                if (p_switch_on && !d_switch_on && !r_switch_on) {
                    last_p_lock_state = g_doorsLocked;
                    g_closeDoorFirstDisplayed = false;
                } else if ((d_switch_on || r_switch_on) && !p_switch_on) {
                    g_doorsLocked = last_p_lock_state;
                    doorState.doors_locked = g_doorsLocked;
                    GPIOPinWrite(DOOR_LOCK_PORT, DOOR_LOCK_PIN, g_doorsLocked ? DOOR_LOCK_PIN : 0);
                }
                xSemaphoreGive(xSpeedMutex);
            }

            if (pressed_count >= 2) {
                snprintf(msg.text, LCD_COLS + 1, "P Task Called");
            } else if (d_switch_on) {
                snprintf(msg.text, LCD_COLS + 1, "Key: D");
                // Clear distance display when switching to D
                msg.command = 1;
                msg.row = 1;
                msg.col = 0;
                snprintf(msg.text, LCD_COLS + 1, "           ");
                SendLCDMessage(&msg, pdMS_TO_TICKS(10));
                msg.command = 1;
                msg.row = 0;
                msg.col = 0;
                snprintf(msg.text, LCD_COLS + 1, "Key: D");
            } else if (p_switch_on) {
                snprintf(msg.text, LCD_COLS + 1, "Key: P");
                // Clear distance display when switching to P
                msg.command = 1;
                msg.row = 1;
                msg.col = 0;
                snprintf(msg.text, LCD_COLS + 1, "           ");
                SendLCDMessage(&msg, pdMS_TO_TICKS(10));
                msg.command = 1;
                msg.row = 0;
                msg.col = 0;
                snprintf(msg.text, LCD_COLS + 1, "Key: P");
                last_p_lock_state = g_doorsLocked;
                g_closeDoorFirstDisplayed = false;
            } else if (r_switch_on) {
                snprintf(msg.text, LCD_COLS + 1, "Key: R");
                if (!door_open) {
                    vTaskResume(sensorTaskHandle);
                    xSemaphoreGive(RstateSemaphore);
                }
            } else {
                snprintf(msg.text, LCD_COLS + 1, "Press Key:");
                // Clear distance display when no gear
                msg.command = 1;
                msg.row = 1;
                msg.col = 0;
                snprintf(msg.text, LCD_COLS + 1, "           ");
                SendLCDMessage(&msg, pdMS_TO_TICKS(10));
                msg.command = 1;
                msg.row = 0;
                msg.col = 0;
                snprintf(msg.text, LCD_COLS + 1, "Press Key:");
                g_closeDoorFirstDisplayed = false;
            }
            SendLCDMessage(&msg, pdMS_TO_TICKS(10));
            g_doorOpenAlert = false;
            vTaskSuspend(buzzerTaskHandle);

            // Suspend sensor task if not in R
            if (!r_switch_on) {
                vTaskSuspend(sensorTaskHandle);
                xSemaphoreTake(RstateSemaphore, pdMS_TO_TICKS(10));
            }
        }
        // Handle ignition off
        else if (switchState.ignition_off && !g_systemStopped) {
            snprintf(msg.text, LCD_COLS + 1, "ignition is off");
            // Clear distance display
            msg.command = 1;
            msg.row = 1;
            msg.col = 0;
            snprintf(msg.text, LCD_COLS + 1, "           ");
            SendLCDMessage(&msg, pdMS_TO_TICKS(10));
            msg.command = 1;
            msg.row = 0;
            msg.col = 0;
            snprintf(msg.text, LCD_COLS + 1, "ignition is off");
            SendLCDMessage(&msg, pdMS_TO_TICKS(10));
            GPIOPinWrite(LED_PORT, GREEN_LED | YELLOW_LED | RED_LED, 0);
            g_doorOpenAlert = false;
            vTaskSuspend(buzzerTaskHandle);
            g_closeDoorFirstDisplayed = false;
            vTaskSuspend(sensorTaskHandle);
            xSemaphoreTake(RstateSemaphore, pdMS_TO_TICKS(10));
        }
        // Handle saraFlag (engine close warning)
        else if (saraFlag && !g_systemStopped) {
            msg.command = 2;
            msg.duration = 0;
            snprintf(msg.text, LCD_COLS + 1, "close engine");
            // Clear distance display
            msg.command = 1;
            msg.row = 1;
            msg.col = 0;
            snprintf(msg.text, LCD_COLS + 1, "           ");
            SendLCDMessage(&msg, pdMS_TO_TICKS(10));
            msg.command = 2;
            msg.row = 0;
            msg.col = 0;
            snprintf(msg.text, LCD_COLS + 1, "close engine");
            SendLCDMessage(&msg, pdMS_TO_TICKS(10));
            GPIOPinWrite(LED_PORT, GREEN_LED | YELLOW_LED | RED_LED, 0);
            g_doorOpenAlert = false;
            vTaskSuspend(buzzerTaskHandle);
            g_closeDoorFirstDisplayed = false;
            vTaskSuspend(sensorTaskHandle);
            xSemaphoreTake(RstateSemaphore, pdMS_TO_TICKS(10));
        }

        xSemaphoreGive(switchStateSemaphore);

        last_d_state = d_switch_on;
        last_p_state = p_switch_on;
        last_r_state = r_switch_on;
        last_door_state = door_open;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Main function
int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    lcdSemaphore = xSemaphoreCreateMutex();
    switchStateSemaphore = xSemaphoreCreateMutex();
    doorStateSemaphore = xSemaphoreCreateMutex();
    xSpeedMutex = xSemaphoreCreateMutex();
    RstateSemaphore = xSemaphoreCreateBinary();
    lcdQueueMutex = xSemaphoreCreateMutex();
    if (lcdSemaphore == NULL || switchStateSemaphore == NULL || doorStateSemaphore == NULL ||
        xSpeedMutex == NULL || lcdQueueMutex == NULL) {
        while (1);
    }

    lcdQueue = xQueueCreate(20, sizeof(LCDMessage_t));
    distanceQueue = xQueueCreate(1, sizeof(float));


    if (lcdQueue == NULL || distanceQueue == NULL ) {
        while (1);
    }

    InitI2C();
    InitLCD();
    InitDoorLock();
    InitSensor();
    InitBuzzer();
    LCDMessage_t msg;
    msg.command = 1;
    msg.row = 0;
    msg.col = 0;
    snprintf(msg.text, LCD_COLS + 1, "ignition is off");
    UpdateLCDBuffer(msg.row, msg.col, msg.text, true, false);
    msg.command = 1;
    msg.row = 1;
    msg.col = 8;
    snprintf(msg.text, LCD_COLS + 1, "D:O L:U");
    UpdateLCDBuffer(msg.row, msg.col, msg.text, true, false);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOInput(SWITCH_PORT, S_KEY_PIN);
    GPIOPadConfigSet(SWITCH_PORT, S_KEY_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    while (GPIOPinRead(SWITCH_PORT, S_KEY_PIN) != 0) {
    }
    msg.command = 0;
    SendLCDMessage(&msg, pdMS_TO_TICKS(10));

    xTaskCreate(LCDTask, "LCDTask", 256, NULL, 1, NULL);
    xTaskCreate(SensorTask, "SensorTask", 256, NULL, 2, &sensorTaskHandle);
    xTaskCreate(BuzzerTask, "BuzzerTask", 256, NULL, 2, &buzzerTaskHandle);
    xTaskCreate(SwitchTask, "SwitchTask", 256, NULL, 4, NULL);
    xTaskCreate(IgnitionSwitchTask, "IgnitionSwitchTask", 256, NULL, 3, NULL);
    xTaskCreate(DoorLockTask, "DoorLockTask", 256, NULL, 3, NULL);
    xTaskCreate(PotentiometerTask, "PotentiometerTask", 256, NULL, 2, NULL);
    vTaskStartScheduler();
    while (1);
}
