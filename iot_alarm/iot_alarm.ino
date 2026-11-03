#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

const int PIN_SDA = 3;
const int PIN_SCL = 2;
const int PIN_CS  = 1;   

// =====================================================================
//  PARAMETRY
// =====================================================================
// --- Algorytm 1: czuwanie / wybudzenie -------------------------------
const int   SLEEP_PERIOD_MS       = 200;    // 5 Hz - niska częstotliwość próbkowania
const float WAKE_THRESHOLD_G      = 0.10f;  // Threshold: odchylenie dowolnej osi X/Y/Z od bazy
const int   WAKE_DURATION_SAMPLES = 2;      // Duration: tyle kolejnych próbek ponad progiem

// --- Algorytm 2: filtrowanie fałszywych alarmów ----------------------
const int   WINDOW_SAMPLES        = 100;    // liczba próbek w oknie
const int   WINDOW_INTERVAL_MS    = 30;     // ~33 Hz -> okno trwa ok. 3 s
const float FILTER_THRESHOLD_G    = 0.15f;  // próg odchylenia całkowitego od pozycji bazowej
const float ALARM_RATIO           = 0.6f;   // min. część próbek ponad progiem = prawdziwy alarm

// --- Pozycja bazowa --------------------------------------------------
const int   CALIB_SAMPLES         = 10;
const int   CALIB_INTERVAL_MS     = 50;
const int   COOLDOWN_MS           = 1000;   // pauza po decyzji, zanim zapamiętamy nową bazę

float baseX = 0, baseY = 0, baseZ = 1;      // wektor przyspieszenia w spoczynku [g]

// =====================================================================
//  ODCZYT AKCELEROMETRU [g]
// =====================================================================
bool readAccel(float &x, float &y, float &z) {
    sensors_event_t event;
    if (!accel.getEvent(&event)) {
        return false;
    }
    // biblioteka zwraca m/s^2 -> przeliczamy na g
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
//  ALGORYTM 1: ZARZĄDZANIE ENERGIĄ (czuwanie)
//  Zwraca true, gdy któraś oś przekracza
//  Threshold nieprzerwanie przez Duration próbek.
// =====================================================================
bool algorithm1_wakeCheck() {
    static int overCount = 0;

    float x, y, z;
    if (!readAccel(x, y, z)) {
        return false;
    }

    float maxAxisDev = fmaxf(fabsf(x - baseX), fmaxf(fabsf(y - baseY), fabsf(z - baseZ)));

    if (maxAxisDev > WAKE_THRESHOLD_G) {
        overCount++;
    } else {
        overCount = 0;
    }

    if (overCount >= WAKE_DURATION_SAMPLES) {
        overCount = 0;
        return true;
    }
    return false;
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

// =====================================================================
void setup() {
    Serial.begin(115200);
    delay(2000); 

    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    delay(10);

    Wire.begin(PIN_SDA, PIN_SCL);

    if (!accel.begin()) {
        Serial.println("Blad: nie wykryto ADXL345. Sprawdz polaczenia!");
        while (true) {
            delay(1000);
        }
    }

    Serial.println("System uruchomiony.");
    calibrateBaseline();
}

void loop() {
    if (algorithm1_wakeCheck()) {
        Serial.println("\n[ALG 1] Wykryto ruch (Threshold + Duration) - wybudzenie.");
        Serial.println("[ALG 2] Start okna pomiarowego.");

        if (algorithm2_isRealAlarm()) {
            Serial.println("[ALG 2] Wynik: PRAWDZIWY ALARM.");
        } else {
            Serial.println("[ALG 2] Wynik: FALSZYWY ALARM.");
        }

        delay(COOLDOWN_MS);
        calibrateBaseline();                 // nowa pozycja bazowa po zdarzeniu
        Serial.println("Powrot do czuwania (5 Hz).\n");
    }
    delay(SLEEP_PERIOD_MS);
}
