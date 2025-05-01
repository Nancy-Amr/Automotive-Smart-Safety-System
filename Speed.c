
int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL |
                   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Enable GPIOF for door switch and buzzer
    SysCtlPeripheralEnable(DOOR_SWITCH_PERIPH);
    while (!SysCtlPeripheralReady(DOOR_SWITCH_PERIPH));
    GPIOPinTypeGPIOInput(DOOR_SWITCH_PORT_BASE, DOOR_SWITCH_PIN);
    GPIOPadConfigSet(DOOR_SWITCH_PORT_BASE, DOOR_SWITCH_PIN,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPinTypeGPIOOutput(BUZZER_PORT_BASE, BUZZER_PIN);

    // Init ADC for potentiometer
    Potentiometer_ADC_Init();

    // Create Mutex and Queue
    xSpeedMutex = xSemaphoreCreateMutex();
    xLCDQueue = xQueueCreate(LCD_QUEUE_SIZE, sizeof(char[64]));
		speedQueue = xQueueCreate(10, sizeof(float));
		if (speedQueue == NULL) {
			while (1); // Failed to create speed queue
		}
    // Tasks
    xTaskCreate(AccelerometerTask, "SpeedInput", 128, NULL, 2, NULL);
    xTaskCreate(DoorBuzzerTask, "Buzzer", 128, NULL, 1, NULL);
    // You should also create an LCD task to receive from xLCDQueue

    vTaskStartScheduler();
    while (1);
}
void PotentiometerTask(void *pvParameters) {
    LCDMessage_t msg;
    Potentiometer_ADC_Init();

    while (1) {
        float speed = ReadSpeedFromPotentiometer();

        // Send to LCD
        msg.command = 1;
        msg.row = 0;
        msg.col = 0;
        snprintf(msg.text, LCD_COLS + 1, "Speed: %.1f km/h", speed);
        xQueueSend(lcdQueue, &msg, portMAX_DELAY);

        // Share with Buzzer task
        xQueueSend(speedQueue, &speed, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


void InitBuzzer(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2); // PF2 for Buzzer
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);        // Initially OFF
}


void DoorBuzzerTask(void *pvParameters)
{
    char lcdMsg[64];
		InitBuzzer();

    while (1)
    {
        int doorOpen = (GPIOPinRead(DOOR_SWITCH_PORT_BASE, DOOR_SWITCH_PIN) == 0);
        float speed = 0.0;

        if (xSemaphoreTake(xSpeedMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            speed = g_vehicleSpeed;

            if (doorOpen && speed > SPEED_THRESHOLD_KMH)
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); // ON
                snprintf(lcdMsg, sizeof(lcdMsg), "Buzzer ON: Door open at %.1f km/h", speed);
                g_vehicleSpeed = 0.0;
            }
            else
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);          // OFF
                snprintf(lcdMsg, sizeof(lcdMsg), "Speed: %.1f km/h | Door %s", speed, doorOpen ? "Open" : "Closed");
            }

            xSemaphoreGive(xSpeedMutex);
            xQueueSend(xLCDQueue, &lcdMsg, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
void Potentiometer_ADC_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5); // PE5 = AIN8

    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0,
                             ADC_CTL_CH8 | ADC_CTL_IE | ADC_CTL_END); // AIN8
    ADCSequenceEnable(ADC0_BASE, 1);
    ADCIntClear(ADC0_BASE, 1);
}

float ReadSpeedFromPotentiometer(void) {
    uint32_t adcValue;
    ADCProcessorTrigger(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC0_BASE, 3, false));
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, &adcValue);

    // Scale ADC (0-4095) to speed (0-30 km/h)
    return (adcValue * 30.0f) / 4095.0f;
}
