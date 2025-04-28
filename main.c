#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "tm4c123gh6pm.h"
#include "project_config.h"
#include "TivaWare_C_Series-2.2.0.295/inc/hw_memmap.h"
#include "TivaWare_C_Series-2.2.0.295/inc/hw_types.h"
#include "TivaWare_C_Series-2.2.0.295/driverlib/sysctl.h"
#include "TivaWare_C_Series-2.2.0.295/driverlib/gpio.h"
#include "TivaWare_C_Series-2.2.0.295/driverlib/pin_map.h"
#include "TivaWare_C_Series-2.2.0.295/driverlib/i2c.h"

// I2C LCD Configuration
#define LCD_I2C_ADDRESS 0x27  // Common I2C address for PCF8574-based LCD
#define LCD_COLUMNS 16
#define LCD_ROWS 2

// LCD Command/Data Flags
#define LCD_RS 0x01   // Register Select (1: Data, 0: Command)
#define LCD_RW 0x02   // Read/Write (0: Write)
#define LCD_EN 0x04   // Enable
#define LCD_BL 0x08   // Backlight (1: On)

// Global FreeRTOS Objects
QueueHandle_t buttonQueue;
SemaphoreHandle_t lcdMutex;

// I2C LCD Functions
void I2C0_Init(void) {
    // Enable I2C0 and GPIO Port B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    
    // Configure PB2 (SCL) and PB3 (SDA) for I2C
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);  // SCL with special handling
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);     // SDA
    
    // Initialize I2C0 as master, standard speed (100 kbps)
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
}

void LCD_SendByte(uint8_t data, uint8_t flags) {
    // Prepare 4-bit data with control bits (RS, RW, Backlight)
    uint8_t high_nibble = (data & 0xF0) | (flags & 0x0F) | LCD_BL;
    uint8_t low_nibble = ((data << 4) & 0xF0) | (flags & 0x0F) | LCD_BL;
    
    // Start I2C transmission
    I2CMasterSlaveAddrSet(I2C0_BASE, LCD_I2C_ADDRESS, false);  // Write mode
    
    // Send high nibble
    I2CMasterDataPut(I2C0_BASE, high_nibble | LCD_EN);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_BASE));
    
    I2CMasterDataPut(I2C0_BASE, high_nibble & ~LCD_EN);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(I2C0_BASE));
    
    // Send low nibble
    I2CMasterDataPut(I2C0_BASE, low_nibble | LCD_EN);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    while(I2CMasterBusy(I2C0_BASE));
    
    I2CMasterDataPut(I2C0_BASE, low_nibble & ~LCD_EN);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C0_BASE));
    
    // Small delay for LCD processing
    SysCtlDelay(150);  // Approx 50us at 80MHz
}

void LCD_Command(uint8_t cmd) {
    LCD_SendByte(cmd, 0);  // RS=0 for command
}

void LCD_Data(uint8_t data) {
    LCD_SendByte(data, LCD_RS);  // RS=1 for data
}

void LCD_Init(void) {
    vTaskDelay(pdMS_TO_TICKS(50));  // Wait for LCD to power up
    
    // Initialize LCD in 4-bit mode
    LCD_SendByte(0x30, 0);  // Wake up
    vTaskDelay(pdMS_TO_TICKS(5));
    LCD_SendByte(0x30, 0);  // Wake up again
    vTaskDelay(pdMS_TO_TICKS(1));
    LCD_SendByte(0x20, 0);  // Set 4-bit mode
    
    // LCD configuration
    LCD_Command(0x28);  // 4-bit, 2 lines, 5x8 font
    LCD_Command(0x08);  // Display off
    LCD_Command(0x01);  // Clear display
    vTaskDelay(pdMS_TO_TICKS(2));
    LCD_Command(0x06);  // Entry mode: increment, no shift
    LCD_Command(0x0C);  // Display on, cursor off, no blink
}

void LCD_Display(const char *str) {
    // Clear display
    LCD_Command(0x01);
    vTaskDelay(pdMS_TO_TICKS(2));
    
    // Write string (up to 16 characters for 16x2 LCD)
    for (int i = 0; str[i] != '\0' && i < LCD_COLUMNS; i++) {
        LCD_Data(str[i]);
    }
}

// Buzzer Simulation (unchanged)
void Buzzer_Beep(uint8_t count, uint32_t duration) {
    // Placeholder for buzzer control (e.g., toggle GPIO pin)
    // Actual implementation depends on hardware setup
}

// Button Monitor Task (unchanged)
void ButtonMonitorTask(void *pvParameters) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    
    uint32_t lastLockButtonState = 1;  // Default to high (not pressed)
    uint32_t lastUnlockButtonState = 1;
    uint32_t lockButtonDebounce = 0;
    uint32_t unlockButtonDebounce = 0;
	
		const uint32_t lockCommand = LOCK_COMMAND;    // Store LOCK_COMMAND value
    const uint32_t unlockCommand = UNLOCK_COMMAND; // Store UNLOCK_COMMAND value
    
    while(1) {
        uint32_t currentLockState = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_0);
        uint32_t currentUnlockState = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_1);
        
        if(currentLockState != lastLockButtonState) {
            lockButtonDebounce++;
            if(lockButtonDebounce >= DEBOUNCE_THRESHOLD) {
                if(currentLockState == 0) {  // Button pressed (active low)
                    xQueueSend(buttonQueue, &lockCommand, 0);
                }
                lastLockButtonState = currentLockState;
                lockButtonDebounce = 0;
            }
        } else {
            lockButtonDebounce = 0;
        }
        
        if(currentUnlockState != lastUnlockButtonState) {
            unlockButtonDebounce++;
            if(unlockButtonDebounce >= DEBOUNCE_THRESHOLD) {
                if(currentUnlockState == 0) {  // Button pressed (active low)
                    xQueueSend(buttonQueue, &unlockCommand, 0);
                }
                lastUnlockButtonState = currentUnlockState;
                unlockButtonDebounce = 0;
            }
        } else {
            unlockButtonDebounce = 0;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // Check buttons every 10ms
    }
}

// Door Control Task (modified for I2C LCD)
void DoorControlTask(void *pvParameters) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);
    
    bool doorsLocked = false;
    uint32_t receivedCommand;
    
    while(1) {
        if(xQueueReceive(buttonQueue, &receivedCommand, portMAX_DELAY) == pdTRUE) {
            if(xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(100))) {
                switch(receivedCommand) {
                    case LOCK_COMMAND:
                        doorsLocked = true;
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_PIN_0);
                        LCD_Display("Doors Locked");
                        break;
                        
                    case UNLOCK_COMMAND:
                        doorsLocked = false;
                        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0);
                        LCD_Display("Doors Unlocked");
                        break;
                }
                xSemaphoreGive(lcdMutex);
            }
            
            if(doorsLocked) {
                Buzzer_Beep(1, 100);
            } else {
                Buzzer_Beep(2, 50);
            }
        }
    }
}

int main(void) {
    // Initialize system clock
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    
    // Initialize I2C and LCD
    I2C0_Init();
    LCD_Init();
    
    // Create FreeRTOS objects
    buttonQueue = xQueueCreate(5, sizeof(uint32_t));
    lcdMutex = xSemaphoreCreateMutex();
    
    // Create tasks
    xTaskCreate(ButtonMonitorTask, "BtnMon", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(DoorControlTask, "DoorCtrl", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
    
    // Start scheduler
    vTaskStartScheduler();
    
    while(1);
}