#include "car_param.h"
#include "sensors.h"
#include "common.h"

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Local definitions               */
/***********************************/
static const unsigned long WAIT = 50;
static const unsigned long WAIT_MONI = 10;

enum {
    SCREEN_OVERVIEW,
    SCREEN_ALTITUDE,
    SCREEN_DPF_STATUS,
    SCREEN_YRP,
    SCREEN_ACC_MONITOR,
    SCREEN_SETTING,
    SCREEN_NUM
};

#define SCREEN_TITLE_BG_RED(x) \
    sprite.fillRect(0, 0, 320, 40, RED); \
    sprite.setTextColor(BLACK); \
    sprite.setFreeFont(&FreeMonoBold12pt7b); \
    sprite.setTextSize(2); \
    sprite.setCursor(20, 35); \
    sprite.printf(x);

#define SCREEN_TITLE(x) \
    sprite.fillRect(0, 0, 320, 40, WHITE); \
    sprite.setTextColor(BLACK); \
    sprite.setFreeFont(&FreeMonoBold12pt7b); \
    sprite.setTextSize(2); \
    sprite.setCursor(20, 35); \
    sprite.printf(x);

#define SET_FONT_AND_SIZE(x, y) \
    sprite.setFreeFont(&x); \
    sprite.setTextSize(y);

/***********************************/
/* Local Variables                 */
/***********************************/
static unsigned long tmr;
static unsigned long tmr_button[3]; //button A,B,C

static unsigned long tmr_save_prefarences;
static const unsigned long SAVE_PREFARENCES_CYCLE = 1000; // 1sec

static int screen;
static float altitude_old;

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
static void btn_process(int button, void func(void)) {
    Button* btn;
    switch (button) {
        case BTN_A:
            btn = &M5.BtnA;
            break;
        case BTN_B:
            btn = &M5.BtnB;
            break;
        case BTN_C:
            btn = &M5.BtnC;
            break;
        default:
            btn = NULL; // kill
            return;
    }
    if (btn->wasPressed() ) {
        func();
    }
    // long press increase/decrease
    if (btn->pressedFor(1000) && tmr_button[button] + 10 < millis() ) {
        tmr_button[button] = millis();
        func();
    }
}

static void display_overview() {
    int percent = 0;
    if (tmr + WAIT < millis()) {
        tmr = millis();

        // display
        sprite.fillScreen(BLACK);
        SCREEN_TITLE("Overview");

        sprite.setTextColor(WHITE);
        SET_FONT_AND_SIZE(FreeMonoBold9pt7b, 1);
        sprite.setCursor(10, 60);
        if (dpf_reg_status) {
            sprite.setTextColor(RED);
        } 
        sprite.printf("DPF");

        sprite.setTextColor(WHITE);
        sprite.setCursor(10, 130);
        sprite.printf("Altitude");

        SET_FONT_AND_SIZE(FreeMono9pt7b, 1);
        sprite.setCursor(15, 80);
        sprite.printf("Accum");
        percent = (int)(dpf_pm_accum / PM_MAX * 100);
        percent = min(percent, 100);
        sprite.drawRect(75, 70, 90, 15, WHITE);
        sprite.fillRect(75, 70, 90 * percent / 100, 15, WHITE);

        sprite.setCursor(15, 100);
        sprite.printf("Gen");
        percent = (int)(dpf_pm_gen / PM_MAX * 100);
        percent = min(percent, 100);
        sprite.drawRect(75, 90, 90, 15, WHITE);
        sprite.fillRect(75, 90, 90 * percent / 100, 15, WHITE);

        SET_FONT_AND_SIZE(FreeMono9pt7b, 1);
        sprite.setCursor(200, 70);
        sprite.printf("Dist");
        SET_FONT_AND_SIZE(FreeMono9pt7b, 2);
        sprite.setCursor(200, 100);
        sprite.printf("%dkm", dpf_reg_dist);


        SET_FONT_AND_SIZE(FreeMono9pt7b, 3);
        sprite.setCursor(15, 180);
        sprite.printf("%dm", (int)altitude);
        
        SET_FONT_AND_SIZE(FreeMono9pt7b, 1);
        sprite.setCursor(140, 150);
        sprite.printf("sea:%.2fhPa", SEALEVELPRESSURE_HPA + sealevel_pressure_offset);
        sprite.setCursor(140, 170);
        sprite.printf("cur:%.2fhPa", (pressure_lps));
        sprite.setCursor(140, 190);
        sprite.printf("temp:%.2fdeg", (temp_lps));
        sprite.setCursor(140, 210);
        sprite.printf("outside:%ddeg", (int)car_outside_temperature);


        sprite.pushSprite(0, 0);
    }
}

static void display_altitude() {
    if (tmr + WAIT < millis()) {
        tmr = millis();

        // display
        sprite.fillScreen(BLACK);
        SCREEN_TITLE("Altitude");

        sprite.setTextColor(WHITE);
        SET_FONT_AND_SIZE(FreeMonoBold12pt7b, 2);
        sprite.setCursor(10, 90);
        sprite.printf("BMP:%dm", (int)altitude);
        sprite.setCursor(10, 150);
        sprite.printf("LPS:%dm", (int)altitude_lps);

        SET_FONT_AND_SIZE(FreeMono9pt7b, 1);
        sprite.setCursor(20, 170);
        sprite.printf("BMP");
        sprite.setCursor(20, 190);
        sprite.printf("sea:%.2fhPa", SEALEVELPRESSURE_HPA + sealevel_pressure_offset);
        sprite.setCursor(20, 210);
        sprite.printf("cur:%.2fhPa", (pressure/100.0));
        sprite.setCursor(20, 230);
        sprite.printf("temp:%.2fdeg", (temp));

        sprite.setCursor(190, 170);
        sprite.printf("LPS");
        sprite.setCursor(190, 190);
        sprite.printf("%.2fhPa", SEALEVELPRESSURE_HPA + sealevel_pressure_offset);
        sprite.setCursor(190, 210);
        sprite.printf("%.2fhPa", (pressure_lps));
        sprite.setCursor(190, 230);
        sprite.printf("%.2fdeg", (temp_lps));


        sprite.pushSprite(0, 0);
    }
}

static void display_dpf_status() {
    int percent = 0;
    if (tmr + WAIT < millis()) {
        tmr = millis();

        sprite.fillScreen(BLACK);
        if (dpf_reg_status) {
            SCREEN_TITLE_BG_RED("DPF status");
        } else {
            SCREEN_TITLE("DPF status");
        }

        sprite.setTextColor(WHITE);
        SET_FONT_AND_SIZE(FreeMono9pt7b, 1);
        sprite.setCursor(10, 75);
        sprite.printf("Acum");
        percent = (int)(dpf_pm_accum / PM_MAX * 100);
        percent = min(percent, 100);
        sprite.drawRect(70, 60, 150, 30, WHITE);
        sprite.fillRect(70, 60, 150 * percent / 100, 30, WHITE);
        sprite.setCursor(230, 75);
        sprite.printf("%.2fg/L", dpf_pm_accum);

        sprite.setCursor(10, 115);
        sprite.printf("Gene");
        percent = (int)(dpf_pm_gen / PM_MAX * 100);
        percent = min(percent, 100);
        sprite.drawRect(70, 100, 150, 30, WHITE);
        sprite.fillRect(70, 100, 150 * percent / 100, 30, WHITE);
        sprite.setCursor(230, 115);
        sprite.printf("%.2fg/L", dpf_pm_gen);

        SET_FONT_AND_SIZE(FreeMono9pt7b, 2);
        sprite.setCursor(10, 170);
        sprite.printf("Count");
        SET_FONT_AND_SIZE(FreeMono9pt7b, 3);
        sprite.setCursor(10, 220);
        sprite.printf("%d", dpf_reg_count);

        SET_FONT_AND_SIZE(FreeMono9pt7b, 2);
        sprite.setCursor(140, 170);
        sprite.printf("Dist");
        SET_FONT_AND_SIZE(FreeMono9pt7b, 3);
        sprite.setCursor(140, 220);
        sprite.printf("%dkm", dpf_reg_dist);

        sprite.pushSprite(0, 0);
    }
}

static void display_yrp() {
    if (tmr + WAIT < millis()) {
        tmr = millis();

        sprite.fillScreen(BLACK);
        SCREEN_TITLE("MPU6050");

        SET_FONT_AND_SIZE(FreeMono9pt7b, 1);
        sprite.setCursor(10, 60);
        sprite.setTextColor(WHITE);
        sprite.printf("Yaw: ");
        sprite.printf("%.2f", yaw);
        sprite.setCursor(10, 90);
        sprite.printf("Roll: ");
        sprite.printf("%.2f", roll);
        sprite.setCursor(10, 120);
        sprite.printf("Pitch: ");
        sprite.printf("%.2f", pitch);


        sprite.setCursor(10, 160);
        sprite.printf("ax: ");
        sprite.printf("%.2f", ax / 16384.0);
        sprite.setCursor(10, 190);
        sprite.printf("ay: ");
        sprite.printf("%.2f", ay / 16384.0);
        sprite.setCursor(10, 220);
        sprite.printf("az: ");
        sprite.printf("%.2f", az / 16384.0);

        sprite.setCursor(160, 160);
        sprite.printf("gx: ");
        sprite.printf("%.2f", gx / 131.0);
        sprite.setCursor(160, 190);
        sprite.printf("gy: ");
        sprite.printf("%.2f", gy / 131.0);
        sprite.setCursor(160, 220);
        sprite.printf("gz: ");
        sprite.printf("%.2f", gz / 131.0);

        sprite.pushSprite(0, 0);
    }
}

static int frame_count = 0;
static int fps = 0;
static int sec = 0;
static void display_acc_monitor() {
    if (tmr + WAIT_MONI < millis()) {
        tmr = millis();
        sprite.fillScreen(BLACK);


        SET_FONT_AND_SIZE(FreeMono9pt7b, 1);
        sprite.setCursor(10, 20);
        sprite.setTextColor(WHITE);
        sprite.printf("FPS: %d", fps);

        int32_t x = (int32_t)(160.0 + (160.0 * ((-ay) / 16384.0)));
        int32_t y = (int32_t)(120.0 + (120.0 * ((az) / 16384.0)));

        sprite.drawCircle(x, y, 10, WHITE);
        frame_count++;
        if (sec != millis() / 1000) {
            fps = frame_count;
            sec = millis() / 1000;
            frame_count = 0;
        }
        sprite.pushSprite(0, 0);
    }
}

static void display_setting() {
    if (tmr + WAIT < millis()) {
        tmr = millis();

        sprite.fillScreen(BLACK);
        SCREEN_TITLE("Setting");
        
        SET_FONT_AND_SIZE(FreeMono9pt7b, 1);
        sprite.setCursor(10, 60);
        sprite.setTextColor(WHITE);
        sprite.printf("Altitude temp from: ");
        sprite.printf("%s", is_temperature_from_sensord?"Sensord":"OBD2");
        
        sprite.setCursor(10, 90);
        sprite.setTextColor(WHITE);
        sprite.printf("Ambient temp: ");
        sprite.printf("%2.2f", car_outside_temperature);
        
    }

    sprite.pushSprite(0, 0);
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
    is_temperature_from_sensord = preferences.getBool("is_temperature_from_sensord", true);

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

    //Initializing LPS25HB
    lps_init();

    //Initializing ELM327
    car_param_init();

    delay(1000); // wait for sensor init
}

void loop() { 
    M5.update();

    switch (screen) {
        case SCREEN_OVERVIEW:
            display_overview();
            btn_process(BTN_A, [](){
                sealevel_pressure_offset += 0.5;
                });
            btn_process(BTN_C, [](){
                sealevel_pressure_offset += -0.5;
                });
            break;
        case SCREEN_ALTITUDE:
            display_altitude();
            btn_process(BTN_A, [](){
                sealevel_pressure_offset += 0.5;
                });
            btn_process(BTN_C, [](){
                sealevel_pressure_offset += -0.5;
                });
            break;
        case SCREEN_DPF_STATUS:
            display_dpf_status();
            break;
        case SCREEN_YRP:
            display_yrp();
            break;
        case SCREEN_ACC_MONITOR:
            display_acc_monitor();
            break;
        case SCREEN_SETTING:
            display_setting();
            btn_process(BTN_A, [](){
                is_temperature_from_sensord = !is_temperature_from_sensord;
                preferences.putBool("is_temperature_from_sensord", is_temperature_from_sensord);
                });
            btn_process(BTN_C, [](){
                is_temperature_from_sensord = !is_temperature_from_sensord;
                preferences.putBool("is_temperature_from_sensord", is_temperature_from_sensord);
                });
            break;
        default:
            break;
    }

    btn_process(BTN_B, [](){
        screen++;
        if (screen >= SCREEN_NUM) {
            screen = SCREEN_OVERVIEW;
        }
        preferences.putInt("screen", screen);
        });

    if ( tmr_save_prefarences + SAVE_PREFARENCES_CYCLE < millis() ) {
        tmr_save_prefarences = millis();
        if ((int)altitude_old != (int)altitude) { // To reduce the number of write acceses to flash memory
            altitude_old = altitude;
            preferences.putFloat("altitude", altitude);
        }
    }
}
