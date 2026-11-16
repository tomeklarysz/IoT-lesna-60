#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "esp_sleep.h"
#include "driver/gpio.h"

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

const int PIN_SDA = 3;
const int PIN_SCL = 2;
const int PIN_CS  = 1;

// --- Wybudzanie / PMU (z adxl-deep-sleep) ----------------------------
const int PIN_WAKE    = 10;   
const int PMU_I2C_SDA = 15;
const int PMU_I2C_SCL = 7;
XPowersPMU PMU;

// =====================================================================
//  PARAMETRY
// =====================================================================
// --- Algorytm 1: czuwanie / wybudzenie -------------------------------
// Wybudza ESP32 z deep sleep, gdy dowolna oś X/Y/Z odchyli się od
// pozycji w chwili zaśnięcia o więcej niż próg.
const float WAKE_THRESHOLD_G      = 0.10f;  // Threshold (rozdzielczość rejestru: 62.5 mg)

// --- Algorytm 2: filtrowanie fałszywych alarmów ----------------------
const int   WINDOW_SAMPLES        = 100;    // liczba próbek w oknie
const int   WINDOW_INTERVAL_MS    = 30;     // ~33 Hz -> okno trwa ok. 3 s
const float FILTER_THRESHOLD_G    = 0.15f;  // próg odchylenia całkowitego od pozycji bazowej
const float ALARM_RATIO           = 0.6f;   // min. część próbek ponad progiem = prawdziwy alarm

// --- Pozycja bazowa --------------------------------------------------
const int   CALIB_SAMPLES         = 10;
const int   CALIB_INTERVAL_MS     = 50;
const int   COOLDOWN_MS           = 1000;   // pauza po decyzji, zanim zapamiętamy nową bazę

// RTC_DATA_ATTR - baza przetrwa deep sleep 
RTC_DATA_ATTR float baseX = 0, baseY = 0, baseZ = 1;   // wektor przyspieszenia w spoczynku [g]

// =====================================================================
//  ODCZYT AKCELEROMETRU [g]
// =====================================================================
bool readAccel(float &x, float &y, float &z) {
    sensors_event_t event;
    if (!accel.getEvent(&event)) {
        return false;
    }
    x = event.acceleration.x / SENSORS_GRAVITY_STANDARD;
    y = event.acceleration.y / SENSORS_GRAVITY_STANDARD;
    z = event.acceleration.z / SENSORS_GRAVITY_STANDARD;
    return true;
}

// Uśrednia kilka próbek w spoczynku i zapamiętuje jako pozycję bazową.
void calibrateBaseline() {
    float sx = 0, sy = 0, sz = 0;
    int n = 0;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        float x, y, z;
        if (readAccel(x, y, z)) {
            sx += x;
            sy += y;
            sz += z;
            n++;
        }
        delay(CALIB_INTERVAL_MS);
    }
    if (n > 0) {
        baseX = sx / n;
        baseY = sy / n;
        baseZ = sz / n;
    }
    Serial.printf("Pozycja bazowa: X=%.2f Y=%.2f Z=%.2f [g]\n", baseX, baseY, baseZ);
}

// =====================================================================
//  ALGORYTM 1: ZARZĄDZANIE ENERGIĄ (czuwanie w deep sleep)
//  ADXL345 sam pilnuje progu i podaje stan wysoki na INT1 -> GPIO10,
//  co wybudza ESP32 (ext0).
// =====================================================================
void goToSleep() {
    uint8_t thresh = (uint8_t)constrain(lroundf(WAKE_THRESHOLD_G / 0.0625f), 1, 255);

    accel.writeRegister(0x2D, 0x00);    // POWER_CTL: standby na czas konfiguracji
    accel.writeRegister(0x24, thresh);  // THRESH_ACT: próg ruchu
    accel.writeRegister(0x27, 0xF0);    // ACT_INACT_CTL: Activity AC, osie X, Y, Z
    accel.writeRegister(0x2F, 0x00);    // INT_MAP: wszystko na INT1
    accel.writeRegister(0x2E, 0x10);    // INT_ENABLE: przerwanie Activity
    accel.writeRegister(0x2D, 0x08);    // POWER_CTL: tryb pomiarowy
    accel.readRegister(0x30);           // INT_SOURCE: skasowanie oczekujących przerwań

    gpio_hold_en((gpio_num_t)PIN_CS);

    Serial.printf("Zasypiam (prog wybudzenia %.3f g).\n\n", thresh * 0.0625f);
    Serial.flush();

    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_WAKE, 1);
    PMU.disableDC3();
    esp_deep_sleep_start();
}

// =====================================================================
//  ALGORYTM 2: FILTROWANIE FAŁSZYWYCH ALARMÓW
//  Przez okno czasowe liczy AccTotal wektora
//  odchylenia od pozycji bazowej: sqrt(dX^2 + dY^2 + dZ^2).
//  Prawdziwy alarm = odchylenie UTRZYMUJE się w większości próbek okna.
// =====================================================================
bool algorithm2_isRealAlarm() {
    int valid = 0, hits = 0;

    for (int i = 0; i < WINDOW_SAMPLES; i++) {
        float x, y, z;
        if (readAccel(x, y, z)) {
            float dx = x - baseX, dy = y - baseY, dz = z - baseZ;
            float accTotal = sqrtf(dx * dx + dy * dy + dz * dz);
            valid++;
            if (accTotal > FILTER_THRESHOLD_G) {
                hits++;
            }
        }
        delay(WINDOW_INTERVAL_MS);
    }

    float ratio = 0.0f;
    if (valid > 0) {
        ratio = (float)hits / valid;
    }

    Serial.printf("Okno: %d/%d próbek ponad progiem (%.0f%%, wymagane %.0f%%)\n",
                  hits, valid, ratio * 100, ALARM_RATIO * 100);

    return ratio >= ALARM_RATIO;
}

void setup() {
    bool wokenByMotion = (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0);

    Serial.begin(115200);
    if (!wokenByMotion) {
        delay(2000);                         
    }

    gpio_hold_dis((gpio_num_t)PIN_CS);
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    delay(10);

    if (PMU.begin(Wire1, AXP2101_SLAVE_ADDRESS, PMU_I2C_SDA, PMU_I2C_SCL)) {
        PMU.disableTSPinMeasure();
    } else {
        Serial.println("Blad zasilania PMU!");
    }

    Wire.begin(PIN_SDA, PIN_SCL);

    if (!accel.begin()) {
        Serial.println("Blad: nie wykryto ADXL345!");
        while (true) {
            delay(1000);
        }
    }

    if (wokenByMotion) {
        Serial.println("\n[ALG 1] Wykryto ruch - wybudzenie.");
        Serial.println("[ALG 2] Start okna pomiarowego.");

        if (algorithm2_isRealAlarm()) {
            Serial.println("[ALG 2] Wynik: PRAWDZIWY ALARM.");
        } else {
            Serial.println("[ALG 2] Wynik: FALSZYWY ALARM.");
        }

        delay(COOLDOWN_MS);
    } else {
        Serial.println("System uruchomiony.");
    }

    calibrateBaseline();                     
    goToSleep();
}

void loop() {
}
