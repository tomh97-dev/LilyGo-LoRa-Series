#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include "boards.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "BLEBeacon.h"
#include "BLEEddystoneTLM.h"
#include "BLEEddystoneURL.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include "ArduinoJson.h"

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

BLEScan* pBLEScan;
int scanTime = 5; //In seconds
uint16_t beconUUID = 0xFEAA;
#define ENDIAN_CHANGE_U16(x) ((((x)&0xFF00)>>8) + (((x)&0xFF)<<8))

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
static const unsigned char bleLogo[] U8X8_PROGMEM = {
    0x00, 0xF0, 0x0F, 0x00, 0x00, 0xFC, 0x3F, 0x00, 0x00, 0xFF, 0xFF, 0x00, 
    0x80, 0x7F, 0xFF, 0x01, 0x80, 0x7F, 0xFE, 0x01, 0xC0, 0x7F, 0xFC, 0x03, 
    0xC0, 0x7F, 0xF8, 0x07, 0xE0, 0x7F, 0xF0, 0x07, 0xE0, 0x7F, 0xE2, 0x07, 
    0xE0, 0x7B, 0xC6, 0x07, 0xE0, 0x71, 0x8E, 0x0F, 0xE0, 0x63, 0xC6, 0x0F, 
    0xF0, 0x07, 0xE2, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x1F, 0xF8, 0x0F, 
    0xF0, 0x3F, 0xF8, 0x0F, 0xF0, 0x3F, 0xFC, 0x0F, 0xF0, 0x1F, 0xF8, 0x0F, 
    0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x07, 0xE2, 0x0F, 0xE0, 0x63, 0xC6, 0x0F, 
    0xE0, 0x71, 0x8E, 0x0F, 0xE0, 0x7B, 0xC6, 0x07, 0xE0, 0x7F, 0xE2, 0x07, 
    0xE0, 0x7F, 0xF0, 0x07, 0xC0, 0x7F, 0xF8, 0x07, 0xC0, 0x7F, 0xFC, 0x03, 
    0x80, 0x7F, 0xFE, 0x01, 0x80, 0x7F, 0xFF, 0x01, 0x00, 0xFF, 0xFF, 0x00, 
    0x00, 0xFC, 0x3F, 0x00, 0x00, 0xF0, 0x0F, 0x00
};

static osjob_t sendjob;
static int spreadFactor = DR_SF7;
static int joinStatus = EV_JOINING;
static const unsigned TX_INTERVAL = 15;
int joiningAnimationIndex = 0;
bool isJoining = false;
unsigned long previousMillis = 0;
const long interval = 500;

static String loraStatus = "";
static String loraDebug = "Awaiting transmission...";
static int fCnt = 0;

static bool scanBLE = true;
static bool scanWiFi = true;

struct BeaconData {
    std::string macAddress;
    int rssi;
};
struct WiFiNetwork {
    String ssid;
    int rssi;
};

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

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    std::vector<BeaconData> beacons;
public:
    void onResult(BLEAdvertisedDevice advertisedDevice) {
    //   Serial.printf("\n\n");
    //   Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
        BeaconData data = {advertisedDevice.getAddress().toString(), advertisedDevice.getRSSI()};
        beacons.push_back(data);
        std::string strServiceData = advertisedDevice.getServiceData();
        uint8_t cServiceData[100];
        strServiceData.copy((char *)cServiceData, strServiceData.length(), 0);

       if (advertisedDevice.getServiceDataUUID().equals(BLEUUID(beconUUID))==true) {  // found Eddystone UUID
        Serial.printf("is Eddystone: %d %s length %d\n", advertisedDevice.getServiceDataUUID().bitSize(), advertisedDevice.getServiceDataUUID().toString().c_str(),strServiceData.length());
        if (cServiceData[0]==0x10) {
           BLEEddystoneURL oBeacon = BLEEddystoneURL();
           oBeacon.setData(strServiceData);
           Serial.printf("Eddystone Frame Type (Eddystone-URL) ");
           Serial.printf(oBeacon.getDecodedURL().c_str());
        } else if (cServiceData[0]==0x20) {
           BLEEddystoneTLM oBeacon = BLEEddystoneTLM();
           oBeacon.setData(strServiceData);
           Serial.printf("Eddystone Frame Type (Unencrypted Eddystone-TLM) \n");
           Serial.printf(oBeacon.toString().c_str());
        } else {
          for (int i=0;i<strServiceData.length();i++) {
            Serial.printf("[%X]",cServiceData[i]);
          }
        }
        Serial.printf("\n");

       } else {
        if (advertisedDevice.haveManufacturerData()==true) {
          std::string strManufacturerData = advertisedDevice.getManufacturerData();
          
          uint8_t cManufacturerData[100];
          strManufacturerData.copy((char *)cManufacturerData, strManufacturerData.length(), 0);
          
          if (strManufacturerData.length()==25 && cManufacturerData[0] == 0x4C  && cManufacturerData[1] == 0x00 ) {
            BLEBeacon oBeacon = BLEBeacon();
            oBeacon.setData(strManufacturerData);
            // Serial.printf("iBeacon Frame\n");
            // Serial.printf("ID: %04X Major: %d Minor: %d UUID: %s Power: %d\n",oBeacon.getManufacturerId(),ENDIAN_CHANGE_U16(oBeacon.getMajor()),ENDIAN_CHANGE_U16(oBeacon.getMinor()),oBeacon.getProximityUUID().toString().c_str(),oBeacon.getSignalPower());
          } else {

            // Serial.printf("strManufacturerData: %d ",strManufacturerData.length());
            for (int i=0;i<strManufacturerData.length();i++) {
            //   Serial.printf("[%X]",cManufacturerData[i]);
            }
            // Serial.printf("\n");
          }
         } else {
        //   Serial.printf("no Beacon Advertised ServiceDataUUID: %d %s \n", advertisedDevice.getServiceDataUUID().bitSize(), advertisedDevice.getServiceDataUUID().toString().c_str());
         }
        }
    }
    void clearBeacons() {
        beacons.clear();
    }
    std::vector<BeaconData> getSortedBeacons() {
        std::sort(beacons.begin(), beacons.end(), [](const BeaconData & a, const BeaconData & b) -> bool {
            return a.rssi > b.rssi;
        });
        return beacons;
    }
};
MyAdvertisedDeviceCallbacks myCallbacks;

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
            // uint16_t symbols[5] = {0x25CB, 0x25D4, 0x25D1, 0x25D5, 0x25CF};
            // uint16_t symbol = symbols[joiningAnimationIndex];
            // u8g2->drawGlyph(64, 32, symbol);
            drawProgressCircle(*u8g2, joiningAnimationIndex);
            u8g2->sendBuffer();
            joiningAnimationIndex = (joiningAnimationIndex + 1) % 5;
        }
    }
}

void showBeaconList()
{
    #ifdef HAS_DISPLAY
        if (u8g2) {
            u8g2->clearBuffer();
            u8g2->setFont(u8g2_font_5x7_tf);
            u8g2->drawStr(0, 12, "BLE Scan Results: ");
            u8g2->drawLine(0, 18, 100, 18);
            auto sortedBeacons = myCallbacks.getSortedBeacons();
            for (int i = 0; i < sortedBeacons.size() && i < 5; i++) {
                String mac = sortedBeacons[i].macAddress.c_str();
                String lastSixMac = mac.substring(mac.length() - 8);
                char buffer[30];
                sprintf(buffer, "MAC: %s RSSI: %d", lastSixMac.c_str(), sortedBeacons[i].rssi);
                u8g2->drawStr(0, 32+(i*8), buffer);
            }
            u8g2->sendBuffer();
        }
    #endif
}

void scanForBLEBeacons() {
    if (!scanBLE) return;
    myCallbacks.clearBeacons();
    // Start BLE scan
    #ifdef HAS_DISPLAY
        if (u8g2) {
            u8g2->clearBuffer();
            u8g2->setFont(u8g2_font_unifont_t_symbols);
            u8g2->drawStr(0, 20, "Scanning for BLE");
            u8g2->drawXBMP(48, 28, 32, 32, bleLogo);
            u8g2->sendBuffer();
        }
    #endif
    Serial.printf("Scanning for BLE...");
    BLEScanResults foundDevices = pBLEScan->start(scanTime);
    Serial.printf("\nScan done! Devices found: %d\n", foundDevices.getCount());

    // Retrieve sorted beacons and print top 5
    auto sortedBeacons = myCallbacks.getSortedBeacons();
    Serial.println("Top 5 Beacons:");
    for (int i = 0; i < sortedBeacons.size() && i < 5; i++) {
        Serial.print("MAC: "); Serial.print(sortedBeacons[i].macAddress.c_str());
        Serial.print(", RSSI: "); Serial.println(sortedBeacons[i].rssi);
        delay(10);
    }

    // Clear results for the next scan
    pBLEScan->clearResults();
}
// void scanForWifi() {
//     #ifdef HAS_DISPLAY
//         if (u8g2) {
//             u8g2->clearBuffer();
//             u8g2->drawStr(0, 12, "Scanning for Wi-Fi...");
//             u8g2->sendBuffer();
//         }
//     #endif
//     std::vector<WiFiNetwork> networks;
//     int n = WiFi.scanNetworks();
//     Serial.println("scan done");
//     if (n == 0) {
//         Serial.println("no networks found");
//     } else {
//         Serial.print(n);
//         Serial.println(" networks found");
//         for (int i = 0; i < n; ++i) {
//             WiFiNetwork network = {
//                 WiFi.SSID(i),
//                 WiFi.RSSI(i)
//             };
//             networks.push_back(network);
//         }

//         // Optional: Sort the networks by RSSI
//         std::sort(networks.begin(), networks.end(), [](const WiFiNetwork& a, const WiFiNetwork& b) {
//             return a.rssi > b.rssi;
//         });

//         // Print the sorted networks
//         for (int i = 0; i < 5; ++i) {
//             Serial.print("MAC: "); Serial.print(networks[i].ssid);
//             Serial.print(", RSSI: "); Serial.println(networks[i].rssi);
//             delay(10);
//         }
//     }
//     WiFi.scanDelete(); // Free up resources by deleting the scan
// }
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
        static uint8_t mydata[] = "Hello, world!";
        static uint8_t myBinaryData[] = {
            0xbb, 0x03, 0x13, 0x48, 0x2d, 0x64, 0x20, 0x9b,
            0x42, 0x96, 0x0e, 0x53, 0x05, 0x00, 0x00, 0x00,
            0x15, 0x02, 0x4a, 0x10, 0x65, 0xd1, 0x81, 0x13,
            0x02, 0xaa, 0x45, 0x13, 0xd1, 0x81, 0x16, 0x02,
            0x90, 0xc7, 0xe7, 0xcf, 0x91, 0x12
        };

        //
        scanForBLEBeacons();
        // scanForWifi();
        // Clear results for the next scan
        pBLEScan->clearResults();
        //
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, myBinaryData, sizeof(myBinaryData), 0);
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);

#ifdef HAS_DISPLAY
        // if (u8g2) {
        //     char buf[256];
        //     u8g2->clearBuffer();
        //     snprintf(buf, sizeof(buf), "[%lu]data sending!", millis() / 1000);
        //     u8g2->drawStr(0, 12, buf);
        //     u8g2->sendBuffer();
        // }
        showBeaconList();
#endif
    }
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
            // loraStatus =  "Received ACK.";
        }
        loraDebug = "RSSI: -" + String(LMIC.rssi) + " SNR: "+ String(LMIC.snr);;

        if (LMIC.dataLen) {
            // data received in rx slot after tx
            Serial.print(F("Data Received: "));
            // Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
            // Serial.println();
            Serial.println(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
        // Schedule next transmission
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
        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).
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

    // Create a JSON array
    JsonArray array = doc.createNestedArray("ble");

    auto sortedBeacons = myCallbacks.getSortedBeacons();
    for (int i = 0; i < sortedBeacons.size() && i < 5; i++) {
        JsonObject device1 = array.createNestedObject();
        device1["id"] = i+1;
        device1["mac"] = sortedBeacons[i].macAddress.c_str();
        device1["rssi"] = sortedBeacons[i].rssi;
        delay(10);
    }

    String output;
    serializeJson(doc, output);
    webSocket.broadcastTXT(output);
}

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
                JsonArray array = doc.createNestedArray("ble");
                JsonObject device1 = array.createNestedObject();
                device1["id"] = 1;
                device1["mac"] = "NO DATA";
                device1["rssi"] = "NO DATA";
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

void setupLMIC(void)
{
    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(spreadFactor, 14);

    Serial.println(F("DevEUI"));
    printArray(DEVEUI, sizeof(DEVEUI) / sizeof(DEVEUI[0]), false);

    // Print APPEUI
    Serial.println(F("APPEUI"));
    printArray(APPEUI, sizeof(APPEUI) / sizeof(APPEUI[0]), false);

    // Print APPKEY
    Serial.println(F("APPKEY"));
    printArray(APPKEY, sizeof(APPKEY) / sizeof(APPKEY[0]), true);

    // Init BLE
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(&myCallbacks);
    pBLEScan->setActiveScan(true);

    //Init Wifi
    // WiFi.mode(WIFI_STA);
    // WiFi.disconnect();

    // Start job
    LMIC_startJoining();

    do_send(&sendjob);     // Will fire up also the join
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
    Serial.println(F("Starting WebServer example as AP..."));

    // Setup ESP32 as an Access Point
    WiFi.mode(WIFI_AP);

    // Set your desired SSID and Password for the Access Point
    const char* ap_ssid = "ESP32-AP";
    const char* ap_password = "12345678";  // Use a stronger password for a real application

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
    server.enableCORS(true);
    server.begin();
}

void initWebsocket()
{
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
}