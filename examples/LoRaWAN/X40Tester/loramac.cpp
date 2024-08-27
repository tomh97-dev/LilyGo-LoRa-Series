#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include "boards.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include "ArduinoJson.h"

#define REGION_EU868 1
#define REGION_US915_0 2
#define REGION_US915_1 3

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// LSB mode
static const u1_t PROGMEM DEVEUI[8] = { 0x2D, 0x13, 0xB2, 0x6C, 0xBE, 0x35, 0xE7, 0x10 };
// LSB mode
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// MSB mode
static const u1_t PROGMEM APPKEY[16] = { 0x75, 0x7F, 0xE4, 0xB5, 0xD0, 0xBB, 0x39, 0x03, 0x9D, 0x24, 0x55, 0x35, 0xD8, 0x42, 0xCE, 0xD7 };

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss =  RADIO_CS_PIN,
    .rxtx = LMIC_UNUSED_PIN,
    .rst =  LMIC_UNUSED_PIN,
    .dio = {RADIO_DIO0_PIN, RADIO_DIO1_PIN, RADIO_BUSY_PIN}
};

static const unsigned char emptyCircle[] U8X8_PROGMEM = {
	0x00, 0xE0, 0x0F, 0x00, 0x00, 0xFC, 0x7F, 0x00, 0x00, 0x1F, 0xF0, 0x01, 
    0x80, 0x03, 0x80, 0x03, 0xE0, 0x00, 0x00, 0x0E, 0x70, 0x00, 0x00, 0x1C, 
    0x30, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x30, 0x0C, 0x00, 0x00, 0x60, 
    0x0C, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0xC0, 0x06, 0x00, 0x00, 0xC0, 
    0x06, 0x00, 0x00, 0xC0, 0x03, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x80, 
    0x03, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x80, 
    0x03, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x80, 0x06, 0x00, 0x00, 0xC0, 
    0x06, 0x00, 0x00, 0xC0, 0x06, 0x00, 0x00, 0xC0, 0x0C, 0x00, 0x00, 0x60, 
    0x0C, 0x00, 0x00, 0x60, 0x18, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x18, 
    0x70, 0x00, 0x00, 0x1C, 0xE0, 0x00, 0x00, 0x0E, 0x80, 0x03, 0x80, 0x03, 
    0x00, 0x1F, 0xF0, 0x01, 0x00, 0xFC, 0x7F, 0x00
};
static const unsigned char circle25[] U8X8_PROGMEM = {
	0x00, 0xE0, 0x0F, 0x00, 0x00, 0xFC, 0x7F, 0x00, 0x00, 0x1F, 0xFF, 0x01, 
    0x80, 0x03, 0xFF, 0x03, 0xE0, 0x00, 0xFF, 0x0F, 0x70, 0x00, 0xFF, 0x1F, 
    0x30, 0x00, 0xFF, 0x1F, 0x18, 0x00, 0xFF, 0x3F, 0x0C, 0x00, 0xFF, 0x7F, 
    0x0C, 0x00, 0xFF, 0x7F, 0x06, 0x00, 0xFF, 0xFF, 0x06, 0x00, 0xFF, 0xFF, 
    0x06, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0xFF, 0xFF, 
    0x03, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x80, 
    0x03, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x80, 0x06, 0x00, 0x00, 0xC0, 
    0x06, 0x00, 0x00, 0xC0, 0x06, 0x00, 0x00, 0xC0, 0x0C, 0x00, 0x00, 0x60, 
    0x0C, 0x00, 0x00, 0x60, 0x18, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x18, 
    0x70, 0x00, 0x00, 0x1C, 0xE0, 0x00, 0x00, 0x0E, 0x80, 0x03, 0x80, 0x03, 
    0x00, 0x1F, 0xF0, 0x01, 0x00, 0xFC, 0x7F, 0x00
};
static const unsigned char circle50[] U8X8_PROGMEM = {
    0x00, 0xE0, 0x0F, 0x00, 0x00, 0xFC, 0x7F, 0x00, 0x00, 0x1F, 0xFF, 0x01, 
    0x80, 0x03, 0xFF, 0x03, 0xE0, 0x00, 0xFF, 0x0F, 0x70, 0x00, 0xFF, 0x1F, 
    0x30, 0x00, 0xFF, 0x1F, 0x18, 0x00, 0xFF, 0x3F, 0x0C, 0x00, 0xFF, 0x7F, 
    0x0C, 0x00, 0xFF, 0x7F, 0x06, 0x00, 0xFF, 0xFF, 0x06, 0x00, 0xFF, 0xFF, 
    0x06, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0xFF, 0xFF, 
    0x03, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0xFF, 0xFF, 
    0x03, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0xFF, 0xFF, 0x06, 0x00, 0xFF, 0xFF, 
    0x06, 0x00, 0xFF, 0xFF, 0x06, 0x00, 0xFF, 0xFF, 0x0C, 0x00, 0xFF, 0x7F, 
    0x0C, 0x00, 0xFF, 0x7F, 0x18, 0x00, 0xFF, 0x3F, 0x30, 0x00, 0xFF, 0x1F, 
    0x70, 0x00, 0xFF, 0x1F, 0xE0, 0x00, 0xFF, 0x0F, 0x80, 0x03, 0xFF, 0x03, 
    0x00, 0x1F, 0xFF, 0x01, 0x00, 0xFC, 0x7F, 0x00
};
static const unsigned char circle75[] U8X8_PROGMEM = {
	0x00, 0xE0, 0x0F, 0x00, 0x00, 0xFC, 0x7F, 0x00, 0x00, 0x1F, 0xFF, 0x01, 
    0x80, 0x03, 0xFF, 0x03, 0xE0, 0x00, 0xFF, 0x0F, 0x70, 0x00, 0xFF, 0x1F, 
    0x30, 0x00, 0xFF, 0x1F, 0x18, 0x00, 0xFF, 0x3F, 0x0C, 0x00, 0xFF, 0x7F, 
    0x0C, 0x00, 0xFF, 0x7F, 0x06, 0x00, 0xFF, 0xFF, 0x06, 0x00, 0xFF, 0xFF, 
    0x06, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0xFF, 0xFF, 
    0x03, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 
    0xFE, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFC, 0xFF, 0xFF, 0x7F, 
    0xFC, 0xFF, 0xFF, 0x7F, 0xF8, 0xFF, 0xFF, 0x3F, 0xF0, 0xFF, 0xFF, 0x1F, 
    0xF0, 0xFF, 0xFF, 0x1F, 0xE0, 0xFF, 0xFF, 0x0F, 0x80, 0xFF, 0xFF, 0x03, 
    0x00, 0xFF, 0xFF, 0x01, 0x00, 0xFC, 0x7F, 0x00
};
static const unsigned char circle100[] U8X8_PROGMEM = {
	0x00, 0xE0, 0x0F, 0x00, 0x00, 0xFC, 0x7F, 0x00, 0x00, 0xFF, 0xFF, 0x01, 
    0x80, 0xFF, 0xFF, 0x03, 0xE0, 0xFF, 0xFF, 0x0F, 0xF0, 0xFF, 0xFF, 0x1F, 
    0xF0, 0xFF, 0xFF, 0x1F, 0xF8, 0xFF, 0xFF, 0x3F, 0xFC, 0xFF, 0xFF, 0x7F, 
    0xFC, 0xFF, 0xFF, 0x7F, 0xFE, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 
    0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 
    0xFE, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFC, 0xFF, 0xFF, 0x7F, 
    0xFC, 0xFF, 0xFF, 0x7F, 0xF8, 0xFF, 0xFF, 0x3F, 0xF0, 0xFF, 0xFF, 0x1F, 
    0xF0, 0xFF, 0xFF, 0x1F, 0xE0, 0xFF, 0xFF, 0x0F, 0x80, 0xFF, 0xFF, 0x03, 
    0x00, 0xFF, 0xFF, 0x01, 0x00, 0xFC, 0x7F, 0x00
};
static const unsigned char circleCheck[] U8X8_PROGMEM = {
    0x00, 0xE0, 0x0F, 0x00, 0x00, 0xFC, 0x7F, 0x00, 0x00, 0xFF, 0xFF, 0x01, 
    0x80, 0xFF, 0xFF, 0x03, 0xE0, 0xFF, 0xFF, 0x0F, 0xF0, 0xFF, 0xFF, 0x1F, 
    0xF0, 0xFF, 0xFF, 0x1F, 0xF8, 0xFF, 0xFF, 0x3F, 0xFC, 0xFF, 0xFF, 0x7F, 
    0xFC, 0xFF, 0xFF, 0x79, 0xFE, 0xFF, 0xFF, 0xF8, 0xFE, 0xFF, 0x7F, 0xFC, 
    0xFE, 0xFF, 0x3F, 0xFE, 0xFF, 0xFF, 0x1F, 0xFF, 0xFF, 0xFF, 0x8F, 0xFF, 
    0xFF, 0xFC, 0xC7, 0xFF, 0xFF, 0xF8, 0xE3, 0xFF, 0xFF, 0xF1, 0xF1, 0xFF, 
    0xFF, 0xE3, 0xF8, 0xFF, 0xFF, 0x47, 0xFC, 0xFF, 0xFE, 0x0F, 0xFE, 0xFF, 
    0xFE, 0x1F, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFC, 0xFF, 0xFF, 0x7F, 
    0xFC, 0xFF, 0xFF, 0x7F, 0xF8, 0xFF, 0xFF, 0x3F, 0xF0, 0xFF, 0xFF, 0x1F, 
    0xF0, 0xFF, 0xFF, 0x1F, 0xE0, 0xFF, 0xFF, 0x0F, 0x80, 0xFF, 0xFF, 0x03, 
    0x00, 0xFF, 0xFF, 0x01, 0x00, 0xFC, 0x7F, 0x00
};

static osjob_t sendjob;
static int spreadFactor = DR_SF7;
static int joinStatus = EV_JOINING;
static const unsigned TX_INTERVAL = 15;
int joiningAnimationIndex = 0;
bool isJoining = false;
unsigned long previousMillis = 0;
const long interval = 500;

int region = REGION_EU868;  // Set this to REGION_EU868, REGION_US915_0 or REGION_US915_1
bool enableADR = true;      // Set to true to enable ADR, false to disable

static String loraStatus = "";
static String loraDebug = "Awaiting transmission...";
static int fCnt = 0;

void os_getArtEui (u1_t *buf)
{
    memcpy_P(buf, APPEUI, 8);
}

void os_getDevEui (u1_t *buf)
{
    memcpy_P(buf, DEVEUI, 8);
}

void os_getDevKey (u1_t *buf)
{
    memcpy_P(buf, APPKEY, 16);
}

void printArray(const u1_t *array, size_t length, bool msbFirst) {
    if (msbFirst) {
        // Print in MSB order
        for (size_t i = 0; i < length; ++i) {
            if (array[i] < 0x10) Serial.print("0"); // Add leading zero for single-digit hex values
            Serial.print(array[i], HEX);
        }
    } else {
        // Print in LSB order (original order)
        for (size_t i = length; i > 0; --i) {
            if (array[i - 1] < 0x10) Serial.print("0"); // Add leading zero for single-digit hex values
            Serial.print(array[i - 1], HEX);
        }
    }
    Serial.println();
}
void drawProgressCircle(U8G2 &u8g2, int progress) {
    const uint8_t *bitmap;
    switch(progress) {
        case 0:
            bitmap = emptyCircle;
            break;
        case 1:
            bitmap = circle25;
            break;
        case 2:
            bitmap = circle50;
            break;
        case 3:
            bitmap = circle75;
            break;
        case 4:
            bitmap = circle100;
            break;
        default:
            bitmap = emptyCircle;
            break;
    }
    u8g2.drawXBMP(48, 28, 32, 32, bitmap);
}
void updateJoiningAnimation() {
    if (!isJoining) {
        return;
    }   
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        if (u8g2) {
            u8g2->clearBuffer();
            u8g2->setFont(u8g2_font_unifont_t_symbols);
            u8g2->drawStr(20, 20, "Joining LNS");
            drawProgressCircle(*u8g2, joiningAnimationIndex);
            u8g2->sendBuffer();
            joiningAnimationIndex = (joiningAnimationIndex + 1) % 5;
        }
    }
}

void sendResponse(uint8_t* data, uint8_t dataLen) {
    LMIC_setTxData2(151, data, dataLen, 0); // Port number 151, no confirmation
}
void do_send(osjob_t *j)
{
    if (joinStatus == EV_JOINING) {
        Serial.println(F("Not joined yet"));
        // Check if there is not a current TX/RX job running
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);

    } else if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        Serial.println(F("OP_TXRXPEND,sending ..."));
        static uint8_t payloadData[] = {
            0xbb, 0x03, 0x13, 0xF2, 0x26, 0x8C, 0xFE, 0x80,
            0xAA, 0xB4, 0x1F, 0xFC, 0x08, 0x00, 0x00, 0x00,
            0x15, 0x02, 0x4a, 0x10, 0x65, 0xd1, 0x81, 0x13,
            0x02, 0xaa, 0x45, 0x13, 0xd1, 0x81, 0x16, 0x02,
            0x90, 0xc7, 0xe7, 0xcf, 0x91, 0x12
        };
        // 0x4F, 0x14 (5 HSD) was 0x53, 0x05

        // GPS COORDS FOR ADNOC
        // Long 0x48 0x2d 0x64 0x20
        // Lat 0x9b, 0x42, 0x96, 0x0e
        // HSD 0x53, 0x05, 0x00, 0x00

        // GPS COORDS FOR OFFICE
        // Latitude Hex: 0x80, 0xAA, 0xB4, 0x1F
        // Longitude Hex: 0xF2, 0x26, 0x8C, 0xFE
        // HSD Hex: 0xFC, 0x08, 0x00, 0x00

        LMIC_setTxData2(1, payloadData, sizeof(payloadData), 0);
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
    }
}

void printFrame(uint8_t* frame, uint8_t length) {
    Serial.print(F("Frame: "));
    for (uint8_t i = 0; i < length; i++) {
        Serial.print(frame[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

void onEvent (ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev) {
    case EV_TXCOMPLETE:
        fCnt++;
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

        if (LMIC.txrxFlags & TXRX_ACK) {
            Serial.println(F("Received ack"));
        }
        loraDebug = "RSSI: -" + String(LMIC.rssi) + " SNR: " + String(LMIC.snr);

        if (LMIC.dataLen) {
            // Data received in rx slot after tx
            Serial.println(F("Data Received: "));
            printFrame(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);

            // Check if the received data is 0x55, 0x01, 0x01 or 0x55, 0x01, 0x00
            if (LMIC.dataLen == 3 && LMIC.frame[LMIC.dataBeg] == 0x55 &&
                LMIC.frame[LMIC.dataBeg + 1] == 0x01 &&
                (LMIC.frame[LMIC.dataBeg + 2] == 0x01 || LMIC.frame[LMIC.dataBeg + 2] == 0x00)) {
                
                // Send the same data back on port 151
                sendResponse(LMIC.frame + LMIC.dataBeg, 3);
            }
        }

        // Schedule next transmission of normal payload
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        break;
    case EV_JOINING:
        isJoining = true;
        digitalWrite(BOARD_LED, LED_OFF);
        Serial.println(F("EV_JOINING: -> Joining..."));
        loraStatus = "OTAA joining....";
        joinStatus = EV_JOINING;
        break;
    case EV_JOIN_FAILED:
        isJoining = false;
        Serial.println(F("EV_JOIN_FAILED: -> Joining failed"));
        loraStatus = "OTAA Joining failed";
#ifdef HAS_DISPLAY
        if (u8g2) {
            u8g2->clearBuffer();
            u8g2->setFont(u8g2_font_unifont_t_symbols);
            u8g2->drawStr(0, 20, "Failed to join LNS!");
            u8g2->drawXBMP(48, 28, 32, 32, circle100);
            u8g2->sendBuffer();
        }
#endif
        break;
    case EV_JOINED:
        isJoining = false;
        digitalWrite(BOARD_LED, LED_ON);
        Serial.println(F("EV_JOINED -> Joined LORIOT!"));
        loraStatus = "Joined";
        joinStatus = EV_JOINED;

#ifdef HAS_DISPLAY
        if (u8g2) {
            u8g2->clearBuffer();
            u8g2->setFont(u8g2_font_unifont_t_symbols);
            u8g2->drawStr(20, 20, "Joined LNS!");
            u8g2->drawXBMP(48, 28, 32, 32, circleCheck);
            u8g2->sendBuffer();
        }
#endif
        delay(3);
        // Disable link check validation (automatically enabled).
        LMIC_setLinkCheckMode(0);

        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    default:
        Serial.println(F("Unknown event"));
        break;
    }
    // SEND WEBSOCKET NOW
    JsonDocument doc;
    doc["loraStatus"] = loraStatus;
    doc["loraDebug"] = loraDebug;
    doc["fCnt"] = LMIC.seqnoUp;
    String output;
    serializeJson(doc, output);
    webSocket.broadcastTXT(output);
}

void setupEU868Channels() {
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band
}

void setupUS915_0Channels() {
    // Setup subband 1 (channels 0-7)
    for (int i = 0; i < 8; i++) {
        LMIC_setupChannel(i, 902300000 + i * 200000, DR_RANGE_MAP(DR_SF10, DR_SF7), BAND_CENTI);
    }
    // Additional channel for RX2 window (optional)
    LMIC_setupChannel(65, 904600000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
}

void setupUS915_1Channels() {
    // Setup subband 2 (channels 8-15)
    for (int i = 0; i < 8; i++) {
        LMIC_setupChannel(i, 903300000 + i * 200000, DR_RANGE_MAP(DR_SF10, DR_SF7), BAND_CENTI);
    }
    // Additional channel for RX2 window (optional)
    LMIC_setupChannel(65, 904600000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
}

void setupLMIC(void) {
    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set clock error
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    // Setup channels based on region
    if (region == REGION_EU868) {
        setupEU868Channels();
        LMIC.dn2Dr = DR_SF9;  // TTN uses SF9 for RX2 in EU868
    } else if (region == REGION_US915_0) {
        setupUS915_0Channels();
        LMIC.dn2Dr = DR_SF12; // SF12 is typically used for RX2 in US915
    } else if (region == REGION_US915_1) {
        setupUS915_1Channels();
        LMIC.dn2Dr = DR_SF12; // SF12 is typically used for RX2 in US915
    }

    Serial.printf("Region: [%u]\n", region);
    // Enable or disable ADR
    LMIC_setAdrMode(enableADR);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // Set data rate and transmit power for uplink
    LMIC_setDrTxpow(spreadFactor, 14);

    // Print LoRaWAN keys
    Serial.println(F("DevEUI"));
    printArray(DEVEUI, sizeof(DEVEUI) / sizeof(DEVEUI[0]), false);

    Serial.println(F("APPEUI"));
    printArray(APPEUI, sizeof(APPEUI) / sizeof(APPEUI[0]), false);

    Serial.println(F("APPKEY"));
    printArray(APPKEY, sizeof(APPKEY) / sizeof(APPKEY[0]), true);

    // Start job
    LMIC_startJoining();

    do_send(&sendjob);
}

// void restartLORA()
// {
//     LMIC.opmode = OP_SHUTDOWN;
//     region = REGION_US915_0;
//     Serial.printf("Restarting!:\n");
//     LMIC_reset();

//     // Set clock error
//     LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

//     // Setup channels based on region
//     if (region == REGION_EU868) {
//         setupEU868Channels();
//         LMIC.dn2Dr = DR_SF9;  // TTN uses SF9 for RX2 in EU868
//     } else if (region == REGION_US915_0) {
//         setupUS915_0Channels();
//         LMIC.dn2Dr = DR_SF12; // SF12 is typically used for RX2 in US915
//     } else if (region == REGION_US915_1) {
//         setupUS915_1Channels();
//         LMIC.dn2Dr = DR_SF12; // SF12 is typically used for RX2 in US915
//     }

//     Serial.printf("Region: [%u]\n", region);
//     // Enable or disable ADR
//     LMIC_setAdrMode(enableADR);

//     // Disable link check validation
//     LMIC_setLinkCheckMode(0);

//     // Set data rate and transmit power for uplink
//     LMIC_setDrTxpow(spreadFactor, 14);

//     // Print LoRaWAN keys
//     Serial.println(F("DevEUI"));
//     printArray(DEVEUI, sizeof(DEVEUI) / sizeof(DEVEUI[0]), false);

//     Serial.println(F("APPEUI"));
//     printArray(APPEUI, sizeof(APPEUI) / sizeof(APPEUI[0]), false);

//     Serial.println(F("APPKEY"));
//     printArray(APPKEY, sizeof(APPKEY) / sizeof(APPKEY[0]), true);

//     // Start job
//     LMIC_startJoining();

//     do_send(&sendjob);
// }

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    JsonDocument doc;
    String output;
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!:\n", num);
            break;
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket.remoteIP(num);
                Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
                 // SEND WEBSOCKET NOW
                doc["loraStatus"] = loraStatus;
                doc["loraDebug"] = loraDebug;
                doc["fCnt"] = LMIC.seqnoUp;
                serializeJson(doc, output);
                webSocket.broadcastTXT(output);
            }
            break;
        case WStype_TEXT:
            Serial.printf("[%u] get Text: %s\n", num, payload);
            break;
        case WStype_BIN:
            Serial.printf("[%u] get Bin: %s\n", num, payload);
            break;
		case WStype_ERROR:			
		case WStype_FRAGMENT_TEXT_START:
		case WStype_FRAGMENT_BIN_START:
		case WStype_FRAGMENT:
		case WStype_FRAGMENT_FIN:
			break;
    }

}

void Task1code(void * pvParameters){
  for(;;){
    server.handleClient();
    webSocket.loop();
    delay(10);
  }
}

void Task2code(void * pvParameters){
  for(;;){
    os_runloop_once();
    updateJoiningAnimation();
    delay(10);
  }
}

void initWIFIAP()
{
    Serial.println(F("Starting WebServer as AP..."));

    // Setup ESP32 as an Access Point
    WiFi.mode(WIFI_AP);

    // Set your desired SSID and Password for the Access Point
    const char* ap_ssid = "LoRa-Tester";
    const char* ap_password = "123456789";  // Use a stronger password for a real application

    // Start the Access Point
    WiFi.softAP(ap_ssid, ap_password);

    // Optional: Set a static IP (if needed)
    // IPAddress local_IP(192, 168, 1, 1);
    // IPAddress gateway(192, 168, 1, 1);
    // IPAddress subnet(255, 255, 255, 0);
    // WiFi.softAPConfig(local_IP, gateway, subnet);

    server.on("/style.css", HTTP_GET, []() {
        File file = SD.open("/bootstrap.min.css");
        if (!file) {
            Serial.println("Failed to open file for reading");
            server.send(404, "text/plain", "File not found");
            return;
        }
        server.streamFile(file, "text/css");
        file.close();
    });
    server.on("/bsbundle.css", HTTP_GET, []() {
        File file = SD.open("/bootstrap.bundle.min.js");
        if (!file) {
            Serial.println("Failed to open file for reading");
            server.send(404, "text/plain", "File not found");
            return;
        }
        server.streamFile(file, "text/css");
        file.close();
    });
    server.on("/icons.css", HTTP_GET, []() {
        File file = SD.open("/icons.fa.css");
        if (!file) {
            Serial.println("Failed to open file for reading");
            server.send(404, "text/plain", "File not found");
            return;
        }
        server.streamFile(file, "text/css");
        file.close();
    });
    server.on("/", HTTP_GET, []() {
        File file = SD.open("/index.html");
        if (!file) {
            Serial.println("Failed to open file for reading");
            server.send(404, "text/plain", "File not found");
            return;
        }
        server.streamFile(file, "text/html");
        file.close();
    });
    server.on("/config", HTTP_GET, []() {
        File file = SD.open("/config.html");
        if (!file) {
            Serial.println("Failed to open file for reading");
            server.send(404, "text/plain", "File not found");
            return;
        }
        server.streamFile(file, "text/html");
        file.close();
    });
    server.on("/restart", HTTP_GET, []() {
        // restartLORA();
        server.send(200, "text/plain", "Restart Complete");
    });
    server.on("/config", HTTP_POST, []() {
        if (!server.hasArg("plain")) {
            server.send(400, "text/plain", "Bad request");
            return;
        }
        String body = server.arg("plain");
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, body);
        if (error) {
            server.send(500, "text/plain", "Failed to parse JSON");
            return;
        }
        // Open file for writing
        File file = SD.open("/config.json", FILE_WRITE);
        if (!file) {
            server.send(500, "text/plain", "Failed to open file for writing");
            return;
        }
        // Serialize JSON to file
        if (serializeJson(doc, file) == 0) {
            server.send(500, "text/plain", "Failed to write to file");
            file.close();
            return;
        }
        file.close();
        server.send(200, "text/plain", "Configuration updated successfully");
    });
    server.on("/getConfig", HTTP_GET, []() {
        File file = SD.open("/config.json");
        if (!file) {
            Serial.println("Failed to open file for reading");
            server.send(404, "text/plain", "File not found");
            return;
        }
        server.streamFile(file, "application/json");
        file.close();
    });
    server.enableCORS(true);
    server.begin();
}

void initWebsocket()
{
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
}