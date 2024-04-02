#include "loramac.h"
#include "boards.h"

void setup()
{
    initBoard();
    delay(1500);
    Serial.println("LoRa Receiver");
    setupLMIC();
}

void loop()
{
    loopLMIC();
}