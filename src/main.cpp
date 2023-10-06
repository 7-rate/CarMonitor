#include "car_param.h"
#include "sensors.h"
#include "common.h"

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Local definitions               */
/***********************************/
enum {
    SCREEN_ALTITUDE,
    SCREEN_DPF_STATUS,
    SCREEN_YRP,
    SCREEN_NUM
};

/***********************************/
/* Local Variables                 */
/***********************************/
static unsigned long tmr;
static const unsigned long WAIT = 100;

static unsigned long tmr_save_prefarences;
static const unsigned long SAVE_PREFARENCES_CYCLE = 1000; // 1sec

int screen;

/***********************************/
/* Global Variables                */
/***********************************/
Preferences preferences;
TFT_eSprite sprite = TFT_eSprite(&M5.Lcd);

/******************************************************************/
/* Implementation                                                 */
/******************************************************************/
/***********************************/
/* Local functions                 */
/***********************************/
static void display_altitude() {
    if (tmr + WAIT < millis()) {
        tmr = millis();

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
        sprite.printf("sea:%dhPa", (int)SEALEVELPRESSURE_HPA + (int)sealevel_pressure_offset);
        sprite.setCursor(20, 200);
        sprite.printf("cur:%dhPa", (int)(pressure/100.0));
        sprite.setCursor(180, 160);
        sprite.printf("temp:%ddeg", (int)(temp));

        sprite.pushSprite(0, 0);
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
    M5.Lcd.setRotation(3);
    preferences.begin("myApp", false);
    screen = preferences.getInt("screen", SCREEN_ALTITUDE);

    sprite.setColorDepth(8);
    sprite.setTextSize(2);
    sprite.createSprite(M5.Lcd.width(), M5.Lcd.height());
    sprite.fillScreen(BLACK);

    //Initializing Serial
    Serial.begin(9600);
    while(!Serial);

    //Initializing BMP280
    bmp_init();

    //Initializing MPU6050
    accelgyro_init();

    //Initializing ELM327
    //car_param_init();

    delay(1000);
}

void loop() { 
    M5.update();

    accelgyro_exec();
    bmp_exec();
    car_param_exec();

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
        sealevel_pressure_offset -= 1.0;
    }
    if (M5.BtnA.wasPressed() ) {
        sealevel_pressure_offset += 1.0;
    }

    if (M5.BtnB.wasPressed() ) {
        screen++;
        if (screen >= SCREEN_NUM) {
            screen = SCREEN_ALTITUDE;
        }
        preferences.putInt("screen", screen);
    }

    if ( tmr_save_prefarences + SAVE_PREFARENCES_CYCLE < millis() ) {
        tmr_save_prefarences = millis();
        preferences.putFloat("altitude", altitude);
    }
}
