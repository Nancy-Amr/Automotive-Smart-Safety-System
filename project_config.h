#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

// Button commands
#define LOCK_COMMAND       1
#define UNLOCK_COMMAND     2

// Debounce settings (5 checks at 10ms interval = 50ms debounce)
#define DEBOUNCE_THRESHOLD 5

// Speed threshold for auto-lock (km/h)
#define SPEED_THRESHOLD    10

// GPIO Pin Definitions
#define LOCK_BUTTON_PIN    GPIO_PIN_0
#define UNLOCK_BUTTON_PIN  GPIO_PIN_1
#define DOOR_LOCK_PIN      GPIO_PIN_0

// GPIO Port Definitions
#define BUTTON_PORT        GPIO_PORTA_BASE
#define LOCK_MECH_PORT     GPIO_PORTB_BASE


#define GPIO_PB2_I2C0SCL 0x00010803
#define GPIO_PB3_I2C0SDA 0x00010C03




#endif // PROJECT_CONFIG_H