#define BLYNK_TEMPLATE_ID "TMPL6gOUQ8BkM"
#define BLYNK_TEMPLATE_NAME "SmokeandGasNotification"
#define BLYNK_AUTH_TOKEN "bELwQXXA3JoIZEdUOHPBO64NjetGMPhp"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <math.h>

// ============================================================
// CẤU HÌNH CHÂN
// ============================================================
#define DHTPIN      D3
#define DHTTYPE     DHT22

#define LED_DO      D7      // LED đỏ cảnh báo gas
#define BUZZER_PIN  D0      // Buzzer kêu 1 lần khi phát hiện gas
#define LED_XANH    D8      // LED xanh khi an toàn

#define MQ2_PIN     A0

// ============================================================
// ADC ESP8266
// Giữ 1023.0 để ADC=1024 cho ra Vout≈3.303V
// ============================================================
#define ADC_REF_COUNT  1023.0
#define ADC_VOLT       3.3

// ============================================================
// MQ-2
// ============================================================
#define RL_MQ2      5.0     // kOhm
#define VCC         5.0     // V
#define Ro_MQ2      1.95

// ============================================================
// AH — Absolute Humidity
// ============================================================
#define AH_CHUAN      2.017
#define AH_MODEL_MIN  0.0
#define AH_MODEL_MAX  2.80

// ============================================================
// THAM SỐ Eq.7 — BÙ ĐỘ ẨM
// Re = ALPHA * exp(-BETA * AH) + C_PARAM
// ============================================================
#define ALPHA       4.54
#define BETA        1.20
#define C_PARAM     0.60

// ============================================================
// THAM SỐ Eq.8 — DÙNG CHO NHÁNH CÓ BÙ
// ppm_comp = K_PARAM * exp(-LAMBDA * (Ros/Ro)) + M_PARAM
// ============================================================
#define K_PARAM     1120.68
#define LAMBDA      0.97
#define M_PARAM     1.99

// ============================================================
// PPM KHÔNG BÙ — POWER LAW ĐÃ TUNE CHO GAS/BUTAN
// Công thức: ppm_nocomp = LPG_A * pow(Rs/Ro, LPG_B)
// Đây là giá trị tuning được khi sử dụng khí gas/butan.
// ============================================================
#define LPG_A       1023.0
#define LPG_B       -2.102

// ============================================================
// TIMING
// ============================================================
#define LOG_INTERVAL_MS   500UL
#define DEBUG_INTERVAL_MS 10000UL
#define DHT_INTERVAL_MS   2500UL
#define WARMUP_MS         60000UL

// ============================================================
// SMOOTH Rs
// ============================================================
#define RS_SMOOTH_WINDOW  5

// ============================================================
// NGƯỠNG CẢNH BÁO ĐỘNG — CHỈ DÙNG PPM CÓ BÙ
//
// Lưu ý:
// - Ngưỡng cảnh báo chỉ đọc ppmComp sau khi thuật toán đã tính xong.
// - Không làm thay đổi Rs, Re, Ros, ratio, ppmComp hoặc ppmNoComp.
// - Chỉ cảnh báo theo nhánh có bù.
// ============================================================
#define ALARM_MIN_PPM        35.0
#define ALARM_OFFSET_PPM     30.0
#define ALARM_MULTIPLIER     2.5
#define ALARM_CLEAR_RATIO    0.75
#define BASELINE_ALPHA       0.02
#define BASELINE_UPDATE_RATE 0.70
#define BEEP_DURATION_MS     200UL


// ============================================================
// KHỞI TẠO DHT VÀ BIẾN TOÀN CỤC
// ============================================================
DHT dht(DHTPIN, DHTTYPE);

float lastTemperature = NAN;
float lastHumidity    = NAN;
bool  dhtHasValidData = false;

unsigned long lastDhtRead = 0;
unsigned long bootTime    = 0;

unsigned long lastLog   = 0;
unsigned long lastDebug = 0;

// ============================================================
// BLYNK WIFI
// ============================================================
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "P501";        // đổi thành tên WiFi của bạn
char pass[] = "phong501";   // đổi thành mật khẩu WiFi của bạn

// ============================================================
// GỬI DỮ LIỆU LÊN BLYNK
// Blynk KHÔNG tính lại bất cứ giá trị nào.
// Chỉ gửi đúng các giá trị đã được tính và in ra Serial log.
// ============================================================
void sendToBlynk(float ppmComp,
                 float ppmNoComp,
                 bool ledRedState,
                 bool ledGreenState) {
    // V0, V1 giữ nguyên: gửi đúng ppm đã được code chính tính xong
    Blynk.virtualWrite(V0, ppmComp);                 // ppm có bù
    Blynk.virtualWrite(V1, ppmNoComp);               // ppm không bù

    // V2, V3 giữ nguyên: trạng thái LED
    Blynk.virtualWrite(V2, ledRedState ? 1 : 0);     // LED đỏ
    Blynk.virtualWrite(V3, ledGreenState ? 1 : 0);   // LED xanh

    // V5, V6 đọc thẳng từ DHT22 tại thời điểm gửi Blynk
    float blynkTemp = dht.readTemperature();
    float blynkHum  = dht.readHumidity();

    if (!isnan(blynkTemp)) {
        Blynk.virtualWrite(V5, blynkTemp);           // nhiệt độ DHT22
    }

    if (!isnan(blynkHum)) {
        Blynk.virtualWrite(V6, blynkHum);            // độ ẩm DHT22
    }
}

float peakPPMComp   = 0.0;
float peakPPMNoComp = 0.0;

// ============================================================
// BIẾN CẢNH BÁO
// ============================================================
float baselinePPMComp = NAN;
float alarmThreshold  = ALARM_MIN_PPM;

bool alarmState     = false;
bool lastAlarmState = false;

unsigned long buzzerOffTime = 0;

// ============================================================
// HÀM CHECK SỐ
// ============================================================
bool isValidNumber(float x) {
    return !isnan(x) && !isinf(x);
}

bool isValidPositive(float x) {
    return isValidNumber(x) && x > 0;
}

float clampFloat(float x, float minVal, float maxVal) {
    if (x < minVal) return minVal;
    if (x > maxVal) return maxVal;
    return x;
}

// ============================================================
// BUZZER NON-BLOCKING
// ============================================================
void beepOnce(unsigned long now) {
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerOffTime = now + BEEP_DURATION_MS;
}

void updateBuzzer(unsigned long now) {
    if (buzzerOffTime > 0 && (long)(now - buzzerOffTime) >= 0) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerOffTime = 0;
    }
}

// ============================================================
// NGƯỠNG CẢNH BÁO ĐỘNG
//
// threshold = max(ALARM_MIN_PPM,
//                 baselinePPMComp * ALARM_MULTIPLIER + ALARM_OFFSET_PPM)
//
// Chỉ dựa trên ppm có bù.
// ============================================================
float calcDynamicAlarmThreshold() {
    if (!isValidPositive(baselinePPMComp)) {
        return ALARM_MIN_PPM;
    }

    float th = baselinePPMComp * ALARM_MULTIPLIER + ALARM_OFFSET_PPM;

    if (!isValidNumber(th) || th < ALARM_MIN_PPM) {
        th = ALARM_MIN_PPM;
    }

    return th;
}

// ============================================================
// CẬP NHẬT NỀN PPM CÓ BÙ
//
// Chỉ học nền khi:
// - hết warmup
// - không đang cảnh báo
// - ppmComp hợp lệ
// - ppmComp còn thấp hơn vùng cảnh báo
// ============================================================
void updateBaselinePPMComp(float ppmComp, bool warmup) {
    if (warmup) return;
    if (!isValidPositive(ppmComp)) return;
    if (alarmState) return;

    if (!isValidPositive(baselinePPMComp)) {
        baselinePPMComp = ppmComp;
        return;
    }

    float currentThreshold = calcDynamicAlarmThreshold();

    if (ppmComp < currentThreshold * BASELINE_UPDATE_RATE) {
        baselinePPMComp =
            baselinePPMComp * (1.0 - BASELINE_ALPHA)
            + ppmComp * BASELINE_ALPHA;
    }
}

// ============================================================
// CẬP NHẬT TRẠNG THÁI CẢNH BÁO
//
// Chỉ dùng ppmComp.
// Không dùng ppmNoComp.
// Không ảnh hưởng đến thuật toán tính ppm.
// ============================================================
void updateAlarmState(float ppmComp, bool warmup, unsigned long now) {
    lastAlarmState = alarmState;

    if (warmup || !isValidPositive(ppmComp)) {
        alarmState = false;
    } else {
        alarmThreshold = calcDynamicAlarmThreshold();

        float clearThreshold = alarmThreshold * ALARM_CLEAR_RATIO;

        if (!alarmState && ppmComp >= alarmThreshold) {
            alarmState = true;
        } else if (alarmState && ppmComp <= clearThreshold) {
            alarmState = false;
        }
    }

    if (warmup) {
        digitalWrite(LED_DO, LOW);
        digitalWrite(LED_XANH, (millis() / 500) % 2 == 0 ? HIGH : LOW);
    } else if (alarmState) {
        digitalWrite(LED_DO, HIGH);
        digitalWrite(LED_XANH, LOW);

        // Buzzer chỉ kêu 1 lần khi vừa chuyển từ SAFE sang GAS DETECTED
        if (!lastAlarmState) {
            beepOnce(now);
        }
    } else {
        digitalWrite(LED_DO, LOW);
        digitalWrite(LED_XANH, HIGH);
    }
}

// ============================================================
// ĐỌC ADC TRUNG BÌNH
// ============================================================
int readMQ2ADC() {
    long sum = 0;

    for (int i = 0; i < 20; i++) {
        sum += analogRead(MQ2_PIN);
        delay(2);
    }

    int adc = sum / 20;

    if (adc < 0) adc = 0;
    if (adc > 1024) adc = 1024;

    return adc;
}

// ============================================================
// ADC -> VOLTAGE
// ADC=1024 vẫn tính bình thường:
// Vout = 1024 * 3.3 / 1023 ≈ 3.303V
// ============================================================
float adcToVoltage(int adc) {
    if (adc <= 0) adc = 1;

    float Vout = adc * (ADC_VOLT / ADC_REF_COUNT);

    if (Vout < 0.01) {
        Vout = 0.01;
    }

    return Vout;
}

// ============================================================
// TÍNH Rs
// Rs = RL * (VCC - Vout) / Vout
// ============================================================
float tinhRsFromADC(int adc, float RL) {
    float Vout = adcToVoltage(adc);

    float Rs = RL * ((VCC - Vout) / Vout);

    if (!isValidPositive(Rs)) {
        return -1;
    }

    return Rs;
}

// ============================================================
// SMOOTH Rs
// ============================================================
float rsWindow[RS_SMOOTH_WINDOW];
int   rsIndex = 0;
int   rsCount = 0;

void initRsSmooth(float firstRs) {
    if (!isValidPositive(firstRs)) {
        firstRs = Ro_MQ2;
    }

    for (int i = 0; i < RS_SMOOTH_WINDOW; i++) {
        rsWindow[i] = firstRs;
    }

    rsCount = RS_SMOOTH_WINDOW;
    rsIndex = 0;
}

float getLastSmoothRs() {
    if (rsCount <= 0) {
        return Ro_MQ2;
    }

    int lastIndex = (rsIndex + RS_SMOOTH_WINDOW - 1) % RS_SMOOTH_WINDOW;
    return rsWindow[lastIndex];
}

float smoothRs(float newRs) {
    if (!isValidPositive(newRs)) {
        return getLastSmoothRs();
    }

    rsWindow[rsIndex] = newRs;
    rsIndex = (rsIndex + 1) % RS_SMOOTH_WINDOW;

    if (rsCount < RS_SMOOTH_WINDOW) {
        rsCount++;
    }

    double sum = 0;

    for (int i = 0; i < rsCount; i++) {
        sum += rsWindow[i];
    }

    return sum / rsCount;
}

// ============================================================
// DHT22
// ============================================================
bool readDHTWithRetry(float &temp, float &hum) {
    for (int i = 0; i < 3; i++) {
        float h = dht.readHumidity();
        float t = dht.readTemperature();

        bool validT = isValidNumber(t) && t > -40.0 && t < 85.0;
        bool validH = isValidNumber(h) && h >= 0.0 && h <= 100.0;

        if (validT && validH) {
            temp = t;
            hum  = h;
            return true;
        }

        delay(100);
    }

    return false;
}

void updateDHT() {
    unsigned long now = millis();

    if (now - lastDhtRead >= DHT_INTERVAL_MS || !dhtHasValidData) {
        lastDhtRead = now;

        float t, h;

        if (readDHTWithRetry(t, h)) {
            lastTemperature = t;
            lastHumidity    = h;
            dhtHasValidData = true;
        }
    }
}

// ============================================================
// AH — Absolute Humidity
// ============================================================
float calcAHRaw(float tempC, float humidity) {
    if (!isValidNumber(tempC) || !isValidNumber(humidity)) {
        return AH_CHUAN;
    }

    humidity = clampFloat(humidity, 0.0, 100.0);

    float denominator = tempC + 237.3;

    if (fabs(denominator) < 0.001) {
        return AH_CHUAN;
    }

    float exponent = (7.5 * tempC) / denominator;

    float AH = (humidity * 6.11 * pow(10.0, exponent)) / 1013.25;

    if (!isValidNumber(AH) || AH < 0) {
        return AH_CHUAN;
    }

    return AH;
}

// ============================================================
// AH USED — GIỚI HẠN VÙNG MODEL
// ============================================================
float calcAHUsedForModel(float AHRaw) {
    if (!isValidNumber(AHRaw) || AHRaw < 0) {
        AHRaw = AH_CHUAN;
    }

    return clampFloat(AHRaw, AH_MODEL_MIN, AH_MODEL_MAX);
}

// ============================================================
// Eq.7 — BÙ ĐỘ ẨM
// Re = ALPHA * exp(-BETA * AH) + C_PARAM
// ============================================================
float tinhReTheoPaper(float AHUsed) {
    if (!isValidNumber(AHUsed) || AHUsed < 0) {
        return -1;
    }

    float Re = ALPHA * exp(-BETA * AHUsed) + C_PARAM;

    if (!isValidPositive(Re)) {
        return -1;
    }

    return Re;
}

// ============================================================
// Eq.8 — TÍNH PPM CÓ BÙ
// ppm = K * exp(-LAMBDA * ratio) + M
// ============================================================
float tinhNongDoTuRatioTheoPaper(float ratio) {
    if (!isValidPositive(ratio)) {
        return -1;
    }

    float ppm = K_PARAM * exp(-LAMBDA * ratio) + M_PARAM;

    if (!isValidNumber(ppm)) {
        return -1;
    }

    if (ppm < 0) {
        ppm = 0;
    }

    return ppm;
}

// ============================================================
// PPM CÓ BÙ
//
// Rs -> AH -> Re -> Ros -> Ros/Ro -> Eq.8 -> ppm_comp
// ============================================================
float tinhNongDoCoBu(float Rs,
                     float AHUsed,
                     float &ReOut,
                     float &RosOut,
                     float &ratioOut) {
    ReOut = tinhReTheoPaper(AHUsed);

    if (!isValidPositive(ReOut)) {
        RosOut   = -1;
        ratioOut = -1;
        return -1;
    }

    RosOut = Rs / ReOut;

    if (!isValidPositive(RosOut) || !isValidPositive(Ro_MQ2)) {
        ratioOut = -1;
        return -1;
    }

    ratioOut = RosOut / Ro_MQ2;

    return tinhNongDoTuRatioTheoPaper(ratioOut);
}

// ============================================================
// PPM KHÔNG BÙ
//
// Không dùng AH.
// Không dùng Re.
// Không dùng Ros.
//
// Chỉ dùng:
// Rs -> Rs/Ro -> ppm_nocomp
// ============================================================
float tinhNongDoKhongBu(float Rs, float &ratioOut) {
    if (!isValidPositive(Rs) || !isValidPositive(Ro_MQ2)) {
        ratioOut = -1;
        return -1;
    }

    ratioOut = Rs / Ro_MQ2;

    if (!isValidPositive(ratioOut)) {
        ratioOut = 0.0001;
    }

    float ppm = LPG_A * pow(ratioOut, LPG_B);

    if (!isValidNumber(ppm)) {
        return -1;
    }

    if (ppm < 0) {
        ppm = 0;
    }

    return ppm;
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(3000);
    Serial.println();

    Blynk.begin(auth, ssid, pass);
    Serial.println("[OK] Blynk connected");

    bootTime = millis();

    pinMode(LED_DO,     OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_XANH,   OUTPUT);

    digitalWrite(LED_DO,     LOW);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_XANH,   LOW);

    dht.begin();

    Serial.println("==================================================");
    Serial.println("   CHE DO LAY DU LIEU — MQ-2 + DHT22");
    Serial.println("   CANH BAO CHI DUNG PPM CO BU");
    Serial.println("   LED DO D7 | BUZZER D0 | LED XANH D8");
    Serial.println("==================================================");

    delay(2000);

    float testT, testRH;

    if (readDHTWithRetry(testT, testRH)) {
        lastTemperature = testT;
        lastHumidity    = testRH;
        dhtHasValidData = true;

        float ahRaw  = calcAHRaw(testT, testRH);
        float ahUsed = calcAHUsedForModel(ahRaw);

        Serial.printf("[OK] DHT22: T=%.1f*C  RH=%.1f%%  AH_raw=%.4f  AH_used=%.4f\n",
                      testT, testRH, ahRaw, ahUsed);
    } else {
        Serial.println("[LOI] DHT22");

        digitalWrite(LED_DO, HIGH);
        digitalWrite(LED_XANH, LOW);

        while (1) {
            digitalWrite(BUZZER_PIN, HIGH);
            delay(150);
            digitalWrite(BUZZER_PIN, LOW);
            delay(850);
        }
    }

    Serial.println("--------------------------------------------------");
    Serial.printf("  Ro_MQ2       : %.2f kOhm\n", (float)Ro_MQ2);
    Serial.printf("  RL_MQ2       : %.2f kOhm\n", (float)RL_MQ2);
    Serial.printf("  VCC          : %.2f V\n",    (float)VCC);
    Serial.printf("  ADC formula  : Vout = ADC * %.1f / %.1f\n",
                  (float)ADC_VOLT, (float)ADC_REF_COUNT);
    Serial.printf("  AH limit     : %.2f -> %.2f\n",
                  (float)AH_MODEL_MIN, (float)AH_MODEL_MAX);
    Serial.printf("  Warm-up      : %lu ms\n", WARMUP_MS);
    Serial.println("--------------------------------------------------");
    Serial.printf("  Eq.7 Re      : %.4f * exp(-%.4f * AH) + %.4f\n",
                  (float)ALPHA, (float)BETA, (float)C_PARAM);
    Serial.printf("  PPM COMP     : %.2f * exp(-%.4f * (Ros/Ro)) + %.3f\n",
                  (float)K_PARAM, (float)LAMBDA, (float)M_PARAM);
    Serial.printf("  PPM NO COMP  : %.2f * (Rs/Ro)^(%.3f)\n",
                  (float)LPG_A, (float)LPG_B);
    Serial.println("--------------------------------------------------");
    Serial.println("  ALARM MODE   : chi dung ppm_comp");
    Serial.printf("  ALARM MIN    : %.2f ppm\n", (float)ALARM_MIN_PPM);
    Serial.printf("  ALARM FORM   : max(%.2f, baseline*%.2f + %.2f)\n",
                  (float)ALARM_MIN_PPM,
                  (float)ALARM_MULTIPLIER,
                  (float)ALARM_OFFSET_PPM);
    Serial.println("--------------------------------------------------");

    delay(1000);

    int firstADC = readMQ2ADC();
    float firstVout = adcToVoltage(firstADC);
    float firstRs = tinhRsFromADC(firstADC, RL_MQ2);

    if (!isValidPositive(firstRs)) {
        Serial.println("[CANH BAO] Rs dau tien khong hop le, dung Ro_MQ2.");
        firstRs = Ro_MQ2;
    }

    initRsSmooth(firstRs);

    Serial.printf("[OK] ADC_RAW  : %d\n", firstADC);
    Serial.printf("[OK] Vout     : %.4f V\n", firstVout);
    Serial.printf("[OK] Rs       : %.2f kOhm\n", firstRs);
    Serial.printf("[OK] Rs/Ro    : %.4f\n", firstRs / Ro_MQ2);

    Serial.println("==================================================");
    Serial.println("FORMAT CSV:");
    Serial.println("ms,Warmup,ADC_RAW,Vout_V,"
                   "Temp_Current_C,Humidity_Current_pct,AH_raw,AH_used,"
                   "Rs_raw,Rs_smooth,"
                   "Re,Ros,Ros_Ro,ppm_comp,peak_comp,"
                   "Rs_Ro_nocomp,ppm_nocomp,peak_nocomp,"
                   "diff_ppm,diff_pct,"
                   "baseline_comp,alarm_threshold,alarm_state");
    Serial.println("--------------------------------------------------");

    digitalWrite(LED_XANH, HIGH);
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    Blynk.run();

    unsigned long now = millis();

    updateBuzzer(now);

    if (now - lastLog < LOG_INTERVAL_MS) {
        delay(10);
        return;
    }

    lastLog = now;

    bool warmup = (now - bootTime < WARMUP_MS);

    updateDHT();

    float AHRaw = dhtHasValidData
                  ? calcAHRaw(lastTemperature, lastHumidity)
                  : AH_CHUAN;

    float AHUsed = calcAHUsedForModel(AHRaw);

    int adcRaw = readMQ2ADC();

    float Vout = adcToVoltage(adcRaw);

    float RsRaw = tinhRsFromADC(adcRaw, RL_MQ2);

    if (!isValidPositive(RsRaw)) {
        RsRaw = getLastSmoothRs();
    }

    float RsSmooth = smoothRs(RsRaw);

    float Re;
    float Ros;
    float ratioComp;
    float ppmComp;

    float ratioNoComp;
    float ppmNoComp;

    // ========================================================
    // THUẬT TOÁN TÍNH PPM — GIỮ NGUYÊN
    // ========================================================
    ppmComp = tinhNongDoCoBu(
        RsSmooth,
        AHUsed,
        Re,
        Ros,
        ratioComp
    );

    ppmNoComp = tinhNongDoKhongBu(
        RsSmooth,
        ratioNoComp
    );

    if (isValidPositive(ppmComp) && ppmComp > peakPPMComp) {
        peakPPMComp = ppmComp;
    }

    if (isValidPositive(ppmNoComp) && ppmNoComp > peakPPMNoComp) {
        peakPPMNoComp = ppmNoComp;
    }

    float diffPPM = ppmNoComp - ppmComp;
    float diffPct = 0.0;

    if (isValidPositive(ppmNoComp)) {
        diffPct = diffPPM / ppmNoComp * 100.0;
    }

    // ========================================================
    // CẢNH BÁO — CHẠY SAU THUẬT TOÁN
    //
    // Chỉ dùng ppmComp.
    // Không ảnh hưởng đến Rs, AH, Re, Ros, ratio hay ppm.
    // ========================================================
    updateAlarmState(ppmComp, warmup, now);
    updateBaselinePPMComp(ppmComp, warmup);

    // ========================================================
    // CSV LOG
    // ========================================================
    Serial.printf("%lu,%d,%d,%.4f,%.2f,%.2f,%.4f,%.4f,%.2f,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n",
        now,
        warmup ? 1 : 0,
        adcRaw,
        Vout,
        dhtHasValidData ? lastTemperature : NAN,
        dhtHasValidData ? lastHumidity    : NAN,
        AHRaw,
        AHUsed,
        RsRaw,
        RsSmooth,
        Re,
        Ros,
        ratioComp,
        ppmComp,
        peakPPMComp,
        ratioNoComp,
        ppmNoComp,
        peakPPMNoComp,
        diffPPM,
        diffPct,
        baselinePPMComp,
        alarmThreshold,
        alarmState ? 1 : 0
    );

    // ========================================================
    // LOG DỄ ĐỌC
    // ========================================================
    Serial.printf("  >> ENV        : T=%.1f*C | RH=%.1f%% | AH_raw=%.4f | AH_used=%.4f | Re=%.4f\n",
        dhtHasValidData ? lastTemperature : NAN,
        dhtHasValidData ? lastHumidity    : NAN,
        AHRaw,
        AHUsed,
        Re
    );

    Serial.printf("  >> ppm co bu   : %.2f ppm  (peak: %.2f)\n",
        ppmComp,
        peakPPMComp
    );

    Serial.printf("  >> ppm khong bu: %.2f ppm  (peak: %.2f)\n",
        ppmNoComp,
        peakPPMNoComp
    );

    Serial.printf("  >> diff        : %.2f ppm  (%.2f%%)\n",
        diffPPM,
        diffPct
    );

    Serial.printf("  >> ALARM       : baseline=%.2f | threshold=%.2f | state=%s\n",
        baselinePPMComp,
        alarmThreshold,
        alarmState ? "GAS DETECTED" : "SAFE"
    );

    // ========================================================
    // BLYNK LOG
    // V0, V1 gửi đúng giá trị ppm đã tính như log.
    // V5, V6 đọc thẳng từ DHT22.
    // ========================================================
    sendToBlynk(
        ppmComp,
        ppmNoComp,
        alarmState,
        !alarmState
    );

    // ========================================================
    // DEBUG MỖI 10 GIÂY
    // ========================================================
    if (now - lastDebug >= DEBUG_INTERVAL_MS) {
        lastDebug = now;

        Serial.println();
        Serial.println("========== DEBUG ==========");
        Serial.printf("Warmup       = %s\n", warmup ? "YES" : "NO");
        Serial.printf("ADC raw      = %d\n", adcRaw);
        Serial.printf("Vout         = %.4f V\n", Vout);

        if (dhtHasValidData) {
            Serial.printf("T            = %.1f *C\n", lastTemperature);
            Serial.printf("RH           = %.1f %%\n", lastHumidity);
            Serial.printf("AH raw       = %.4f\n", AHRaw);
            Serial.printf("AH used      = %.4f\n", AHUsed);
        } else {
            Serial.printf("DHT22 chua co du lieu, dung AH_CHUAN=%.4f\n", AH_CHUAN);
        }

        Serial.printf("Rs raw       = %.2f kOhm\n", RsRaw);
        Serial.printf("Rs smooth    = %.2f kOhm\n", RsSmooth);

        Serial.println("----- CO BU -----");
        Serial.printf("Re           = %.4f\n", Re);
        Serial.printf("Ros          = %.4f kOhm\n", Ros);
        Serial.printf("Ros/Ro       = %.4f\n", ratioComp);
        Serial.printf("PPM COMP     = %.2f\n", ppmComp);
        Serial.printf("PEAK COMP    = %.2f\n", peakPPMComp);

        Serial.println("----- KHONG BU -----");
        Serial.printf("Rs/Ro        = %.4f\n", ratioNoComp);
        Serial.printf("PPM NO COMP  = %.2f\n", ppmNoComp);
        Serial.printf("PEAK NO COMP = %.2f\n", peakPPMNoComp);
        Serial.printf("Formula      = %.2f * (Rs/Ro)^(%.3f)\n",
                      (float)LPG_A, (float)LPG_B);
        Serial.println("Note         = LPG_A/LPG_B tuning khi dung gas/butan");

        Serial.println("----- CANH BAO -----");
        Serial.println("Alarm source = ppm_comp only");
        Serial.printf("Baseline     = %.2f ppm\n", baselinePPMComp);
        Serial.printf("Threshold    = %.2f ppm\n", alarmThreshold);
        Serial.printf("Alarm state  = %s\n", alarmState ? "GAS DETECTED" : "SAFE");
        Serial.printf("LED RED D7   = %s\n", alarmState ? "ON" : "OFF");
        Serial.printf("LED GREEN D8 = %s\n", alarmState ? "OFF" : "ON");

        Serial.println("----- SO SANH -----");
        Serial.printf("Diff         = %.2f ppm\n", diffPPM);
        Serial.printf("Diff pct     = %.2f %%\n", diffPct);

        Serial.println("===========================");
        Serial.println();
    }
}
