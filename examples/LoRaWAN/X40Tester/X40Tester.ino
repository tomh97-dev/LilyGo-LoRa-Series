#include "loramac.h"
#include "boards.h"

TaskHandle_t Task1;
TaskHandle_t Task2;

void setup()
{
    initBoard();
    delay(1500);
    Serial.println("LoRa Receiver");
    setupLMIC();
    initWIFIAP();
    initWebsocket();

    xTaskCreatePinnedToCore(
                Task1code,   /* Task function. */
                "Task1",     /* Name of the task. */
                10000,       /* Stack size of task */
                NULL,        /* parameter of the task */
                1,           /* priority of the task */
                &Task1,      /* Task handle to keep track of created task */
                0);          /* pin task to core 0 */                  
    delay(500); 

    xTaskCreatePinnedToCore(
                Task2code,   /* Task function. */
                "Task2",     /* Name of the task. */
                10000,       /* Stack size of task */
                NULL,        /* parameter of the task */
                1,           /* priority of the task */
                &Task2,      /* Task handle to keep track of created task */
                1);          /* pin task to core 1 */
    delay(500); 
}

void loop() {
    // Empty loop
    delay(1000); // A delay to prevent it from spinning.
    // Or you can add non-critical background tasks here if needed.
}