#include <M5Stack.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <BluetoothSerial.h>
#include <ELMduino.h>
#include <Preferences.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Local definitions               */
/***********************************/
#define SEALEVELPRESSURE_HPA (1013.25)
#define TEMP_OFFSET (-2.5)
#define OBD_TIMEOUT (1000)

enum {
    SCREEN_ALTITUDE,
    SCREEN_DPF_STATUS,
    SCREEN_YRP,
    SCREEN_NUM
};

// for Mazda2 PIDs
const uint16_t DPF_PM_ACCUMULATION = 0x042C;
const uint16_t DPF_PM_GENERATION = 0x042D;
const uint16_t DPF_REGENERATION_COUNT = 0x0432;
const uint16_t DPF_REGENERATION_DISTANCE = 0x0434;
const uint16_t DPF_REGENERATION_STATUS = 0x0380;

#define PM_MAX (6.0)

// MPU6050 offsets
#define MPU6050_XA_OFFSET (-3164 )
#define MPU6050_YA_OFFSET ( 348 )
#define MPU6050_ZA_OFFSET ( 2009 )
#define MPU6050_XG_OFFSET ( -49 )
#define MPU6050_YG_OFFSET ( -15 )
#define MPU6050_ZG_OFFSET ( -61 )

/***********************************/
/* Local Variables                 */
/***********************************/
uint8_t ELM327_MACADDRESS[] = {0xAA,0xBB,0xCC,0x11,0x22,0x33};

Adafruit_BMP280 bmp;
ELM327 elm;
BluetoothSerial SerialBT;
Preferences preferences;
TFT_eSprite sprite = TFT_eSprite(&M5.Lcd);

static int pressure_offset = 0;
static unsigned long tmr;
static const unsigned long WAIT = 100;
static bool flag_use_bt = false;

int screen = SCREEN_ALTITUDE;

uint32_t rpm;
uint32_t kph;
float dpf_pm_accum;
float dpf_pm_gen;
int dpf_reg_count;
int dpf_reg_dist;
int dpf_reg_status;

static unsigned long tmr_obd_timeout;


MPU6050 mpu(0x68);
int16_t ax, ay, az;
int16_t gx, gy, gz;
float roll, pitch, yaw;
Madgwick madgwickfilter;


/***********************************/
/* Global Variables                */
/***********************************/

/******************************************************************/
/* Implementation                                                 */
/******************************************************************/
/***********************************/
/* Local functions                 */
/***********************************/
static void display_altitude() {
    if (tmr + WAIT < millis() && bmp.takeForcedMeasurement()) {
        tmr = millis();
        float temp = bmp.readTemperature() + TEMP_OFFSET;
        float pressure = bmp.readPressure();
        float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA + (float)pressure_offset);

        // display
        sprite.fillScreen(BLACK);
        sprite.fillRect(0, 0, 320, 40, WHITE);
        sprite.setTextSize(3);
        sprite.setTextColor(BLACK);
        sprite.setCursor(20, 8);
        sprite.printf("Altitude");

        sprite.setTextSize(7);
        sprite.setTextColor(WHITE);
        sprite.setCursor(50, 70);
        sprite.printf("%dm", (int)altitude);

        sprite.setTextSize(2);
        sprite.setCursor(20, 160);
        sprite.printf("sea:%dhPa", (int)SEALEVELPRESSURE_HPA + pressure_offset);
        sprite.setCursor(20, 200);
        sprite.printf("cur:%dhPa", (int)(pressure/100));
        sprite.setCursor(180, 160);
        sprite.printf("temp:%ddeg", (int)(temp + TEMP_OFFSET));

        sprite.pushSprite(0, 0);
    }
}

static void update_dpf_pm_accumulation() {
    float temp = elm.processPID(0x22, DPF_PM_ACCUMULATION, 1, 2, 100.0, 0.0); //(A*256+B)*100/65535
    tmr_obd_timeout = millis();
    while(elm.nb_rx_state != ELM_SUCCESS && millis() - tmr_obd_timeout < OBD_TIMEOUT) {
        temp = elm.processPID(0x22, DPF_PM_ACCUMULATION, 1, 2, 100.0/65535.0, 0.0);
    }
    dpf_pm_accum = temp;
}

static void update_dpf_pm_generation() {
    float temp = elm.processPID(0x22, DPF_PM_GENERATION, 1, 2, 100.0, 0.0); //(A*256+B)*100/65535
    tmr_obd_timeout = millis();
    while(elm.nb_rx_state != ELM_SUCCESS && (long)(millis() - tmr_obd_timeout) < OBD_TIMEOUT) {
        temp = elm.processPID(0x22, DPF_PM_GENERATION, 1, 2, 0.00153, 0.0);
    }
    dpf_pm_gen = temp;
}

static void update_dpf_regeneration_count() {
    int temp = (int)elm.processPID(0x22, DPF_REGENERATION_COUNT, 1, 2, 1.0, 0.0); //A*256+B
    tmr_obd_timeout = millis();
    while(elm.nb_rx_state != ELM_SUCCESS && (long)(millis() - tmr_obd_timeout) < OBD_TIMEOUT) {
        temp = (int)elm.processPID(0x22, DPF_REGENERATION_COUNT, 1, 2, 1.0, 0.0);
    }
    dpf_reg_count = temp;
}

static void update_dpf_regeneration_distance() {
    int temp = (int)elm.processPID(0x22, DPF_REGENERATION_DISTANCE, 1, 4, 1.0, 0.0); //((B<<16)+(C<<8)+D)/640
    tmr_obd_timeout = millis();
    while(elm.nb_rx_state != ELM_SUCCESS && (long)(millis() - tmr_obd_timeout) < OBD_TIMEOUT) {
        temp = (int)elm.processPID(0x22, DPF_REGENERATION_DISTANCE, 1, 4, 0.00156, 0.0);
    }
    dpf_reg_dist = temp;
}

static void update_dpf_regeneration_status() {
    int temp = (int)elm.processPID(0x22, DPF_REGENERATION_STATUS, 1, 1, 1.0, 0.0); //(A)
    tmr_obd_timeout = millis();
    while(elm.nb_rx_state != ELM_SUCCESS && (long)(millis() - tmr_obd_timeout) < OBD_TIMEOUT) {
        temp = (int)elm.processPID(0x22, DPF_REGENERATION_STATUS, 1, 1, 1.0, 0.0);
    }
    dpf_reg_status = temp;
}

void task_update_obd(void* arg) {
    while(1) {
        update_dpf_regeneration_status();
        update_dpf_pm_accumulation();
        update_dpf_pm_generation();
        update_dpf_regeneration_count();
        update_dpf_regeneration_distance();
    }
}

static void display_dpf_status() {
    int percent = 0;
    if (tmr + WAIT < millis()) {
        tmr = millis();

        sprite.fillScreen(BLACK);
        sprite.setTextSize(3);
        sprite.setCursor(20, 8);
        sprite.fillRect(0, 0, 320, 40, dpf_reg_status ? RED : WHITE);
        sprite.setTextColor(BLACK);
        sprite.printf("DPF Status:%d", dpf_reg_status);

        sprite.setCursor(10, 65);
        sprite.setTextSize(2);
        sprite.setTextColor(WHITE);
        sprite.printf("Acum");
        percent = (int)(dpf_pm_accum / PM_MAX * 100);
        percent = min(percent, 100);
        sprite.drawRect(70, 60, 150, 30, WHITE);
        sprite.fillRect(70, 60, 150 * percent / 100, 30, WHITE);
        sprite.setCursor(230, 65);
        sprite.printf("%.2fg/L", dpf_pm_accum);

        sprite.setCursor(10, 105);
        sprite.printf("Gene");
        percent = (int)(dpf_pm_gen / PM_MAX * 100);
        percent = min(percent, 100);
        sprite.drawRect(70, 100, 150, 30, WHITE);
        sprite.fillRect(70, 100, 150 * percent / 100, 30, WHITE);
        sprite.setCursor(230, 105);
        sprite.printf("%.2fg/L", dpf_pm_gen);

        sprite.setCursor(20, 160);
        sprite.setTextSize(2);
        sprite.printf("Count");
        sprite.setTextSize(4);
        sprite.setCursor(20, 190);
        sprite.printf("%d", dpf_reg_count);

        sprite.setCursor(180, 160);
        sprite.setTextSize(2);
        sprite.printf("Dist");
        sprite.setTextSize(4);
        sprite.setCursor(180, 190);
        sprite.printf("%dkm", dpf_reg_dist);

        sprite.pushSprite(0, 0);
    }
}

static void display_yrp() {
    if (tmr + WAIT < millis()) {
        tmr = millis();

        sprite.fillScreen(BLACK);
        sprite.setTextSize(3);
        sprite.setCursor(20, 8);
        sprite.fillRect(0, 0, 320, 40, WHITE);
        sprite.setTextColor(BLACK);
        sprite.printf("YawRollPitch");

        sprite.setCursor(10, 50);
        sprite.setTextSize(2);
        sprite.setTextColor(WHITE);
        sprite.printf("Yaw: ");
        sprite.printf("%.2f", yaw);

        sprite.setCursor(10, 80);
        sprite.printf("Roll: ");
        sprite.printf("%.2f", roll);

        sprite.setCursor(10, 110);
        sprite.printf("Pitch: ");
        sprite.printf("%.2f", pitch);


        sprite.setCursor(10, 150);
        sprite.printf("ax: ");
        sprite.printf("%.2f", ax / 16384.0);

        sprite.setCursor(10, 180);
        sprite.printf("ay: ");
        sprite.printf("%.2f", ay / 16384.0);

        sprite.setCursor(10, 210);
        sprite.printf("az: ");
        sprite.printf("%.2f", az / 16384.0);

        sprite.setCursor(160, 150);
        sprite.printf("gx: ");
        sprite.printf("%.2f", gx / 131.0);

        sprite.setCursor(160, 180);
        sprite.printf("gy: ");
        sprite.printf("%.2f", gy / 131.0);

        sprite.setCursor(160, 210);
        sprite.printf("gz: ");
        sprite.printf("%.2f", gz / 131.0);

        sprite.pushSprite(0, 0);
    }
}

/***********************************/
/* Global functions                */
/***********************************/
void setup() {
    bool error = false;

    M5.begin();

    flag_use_bt = !M5.BtnB.isPressed();

    sprite.setColorDepth(8);
    sprite.setTextSize(2);
    sprite.createSprite(M5.Lcd.width(), M5.Lcd.height());
    M5.Lcd.setRotation(3);
    sprite.fillScreen(BLACK);

    //Initializing Serial
    Serial.begin(9600);
    while(!Serial);

    //Initializing BMP280
    sprite.setCursor(10, 20);
    sprite.print("Init BMP280...");
    sprite.pushSprite(0, 0);
    error = false;
    if (!bmp.begin(0x76)) {
        Serial.println(F("Error BMP280"));
        error = true;
    }
    if (!error) {
        bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    }
    sprite.setCursor(240, 20);
    sprite.print("done!");
    sprite.pushSprite(0, 0);

    //Initializing MPU6050
    sprite.setCursor(10, 50);
    sprite.printf("Init MPU6050...");
    sprite.pushSprite(0, 0);
    error = false;
    mpu.initialize();
    mpu.setXAccelOffset(MPU6050_XA_OFFSET);
    mpu.setYAccelOffset(MPU6050_YA_OFFSET);
    mpu.setZAccelOffset(MPU6050_ZA_OFFSET);
    mpu.setXGyroOffset(MPU6050_XG_OFFSET);
    mpu.setYGyroOffset(MPU6050_YG_OFFSET);
    mpu.setZGyroOffset(MPU6050_ZG_OFFSET);

    sprite.setCursor(240, 50);
    sprite.print("done!");
    sprite.pushSprite(0, 0);

    //Initializing Madgwick
    madgwickfilter.begin(100); //sampling 100Hz

    //Connecting ELM327
    if (flag_use_bt) {
        sprite.setCursor(10, 80);
        sprite.printf("Init ELM327 phase1...");
        sprite.pushSprite(0, 0);
        error = false;
        SerialBT.begin("ArduHUD", true);
        if (!SerialBT.connect(ELM327_MACADDRESS)) {
            Serial.println("Cant connect to OBD - pahse1");
            error = true;
        }
        sprite.setCursor(240, 80);
        sprite.print(error?"err!!":"done!");
        sprite.pushSprite(0, 0);

        sprite.setCursor(10, 110);
        sprite.printf("Init ELM327 phase2...");
        sprite.pushSprite(0, 0);
        error = false;
        if (!elm.begin(SerialBT, true, 2000)) {
            Serial.println("Cant connect to OBD - phase2");
            error = true;
        }
    }
    sprite.setCursor(240, 110);
    sprite.print(error?"err!!":"done!");
    sprite.pushSprite(0, 0);

    sprite.setTextSize(4);
    sprite.setCursor(10, 180);
    sprite.print(error?"Init Error!":"Init Complete!");
    sprite.pushSprite(0, 0);

    preferences.begin("myApp", false);
    pressure_offset = preferences.getInt("pressure_offset", 0);

    xTaskCreatePinnedToCore(task_update_obd, "task_update_obd", 4096, NULL, 1, NULL, 0);
    delay(1000);
}

void loop() { 
    M5.update();

    // update sensor
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    madgwickfilter.updateIMU(gx / 131.0, gy / 131.0, gz / 131.0, ax / 16384.0, ay / 16384.0, az / 16384.0);
    roll = madgwickfilter.getRoll();
    pitch = madgwickfilter.getPitch();
    yaw = madgwickfilter.getYaw();

    // process display
    switch (screen) {
        case SCREEN_ALTITUDE:
            display_altitude();
            break;
        case SCREEN_DPF_STATUS:
            display_dpf_status();
            break;
        case SCREEN_YRP:
            display_yrp();
            break;
        default:
            break;
    }

    // process button
    if (M5.BtnC.wasPressed() ) {
        pressure_offset--;
        preferences.putInt("pressure_offset", pressure_offset);
    }
    if (M5.BtnA.wasPressed() ) {
        pressure_offset++;
        preferences.putInt("pressure_offset", pressure_offset);
    }

    if (M5.BtnB.wasPressed() ) {
        screen++;
        if (screen >= SCREEN_NUM) {
            screen = SCREEN_ALTITUDE;
        }
    }
}
