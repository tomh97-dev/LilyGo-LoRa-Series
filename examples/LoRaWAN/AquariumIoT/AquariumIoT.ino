#include "loramac.h"
#include "boards.h"

void setup()
{
    initBoard();
    delay(1500);
    Serial.println("LNS Connector");
    setupLMIC();
    initWIFIAP();
}

void loop()
{
    loopLMIC();
}