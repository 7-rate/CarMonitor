#include "car_param.h"
#include "sensors.h"
#include "common.h"
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

/******************************************************************/
/* Definitions                                                    */
/******************************************************************/
/***********************************/
/* Local definitions               */
/***********************************/
static const unsigned long WAIT = 10;
static const unsigned long WAIT_MONI = 10;

// clang-format off
enum { 
    SCREEN_OVERVIEW, 
    // SCREEN_ALTITUDE,
    SCREEN_DPF_STATUS,
    SCREEN_ADDITIONAL_METER,
    SCREEN_SETTING,
    SCREEN_NUM
};
// clang-format on

#define SCREEN_TITLE_BG_RED( x )                                                                                       \
    sprite.fillRect( 0, 0, 320, 20, RED );                                                                             \
    sprite.setTextColor( BLACK );                                                                                      \
    sprite.setFreeFont( &FreeMonoBold9pt7b );                                                                          \
    sprite.setTextSize( 1 );                                                                                           \
    sprite.setCursor( 10, 10 );                                                                                        \
    sprite.printf( x );

#define SCREEN_TITLE( x )                                                                                              \
    sprite.fillRect( 0, 0, 320, 20, WHITE );                                                                           \
    sprite.setTextColor( BLACK );                                                                                      \
    sprite.setFreeFont( &FreeMonoBold9pt7b );                                                                          \
    sprite.setTextSize( 1 );                                                                                           \
    sprite.setCursor( 10, 12 );                                                                                        \
    sprite.printf( x );

#define SET_FONT_AND_SIZE( x, y )                                                                                      \
    sprite.setFreeFont( &x );                                                                                          \
    sprite.setTextSize( y );

TinyGPSPlus gps;
SoftwareSerial ss( 26, 25 );

/***********************************/
/* Local Variables                 */
/***********************************/
static unsigned long tmr;
static unsigned long tmr_button[3]; // button A,B,C

static unsigned long tmr_save_prefarences;
static const unsigned long SAVE_PREFARENCES_CYCLE = 1000; // 1sec

static int screen;
static float altitude_old;

// デバッグモード用変数
static bool debug_mode = false;
static String serial_command = "";

/***********************************/
/* Global Variables                */
/***********************************/
Preferences preferences;
TFT_eSprite sprite = TFT_eSprite( &M5.Lcd );

/******************************************************************/
/* Implementation                                                 */
/******************************************************************/
/***********************************/
/* Local functions                 */
/***********************************/
// デバッグモード：シリアルコマンド処理
static void process_debug_command( String cmd ) {
    cmd.trim();
    cmd.toLowerCase();

    if ( cmd == "help" || cmd == "h" ) {
        Serial.println( "\n=== Debug Commands ===" );
        Serial.println( "help/h              - Show this help" );
        Serial.println( "debug on/off        - Enable/Disable debug mode" );
        Serial.println( "dpf <value>         - Set DPF PM accumulation (0-10 g/L)" );
        Serial.println( "dpfgen <value>      - Set DPF PM generation (0-10 g/L)" );
        Serial.println( "dpfdist <value>     - Set DPF regeneration distance (km)" );
        Serial.println( "dpfcount <value>    - Set DPF regeneration count" );
        Serial.println( "dpfstatus <0/1>     - Set DPF regeneration status" );
        Serial.println( "water <value>       - Set water temperature (deg)" );
        Serial.println( "oil <value>         - Set oil temperature (deg)" );
        Serial.println( "fuel <value>        - Set fuel level (0-100 %)" );
        Serial.println( "pressure <value>    - Set pressure (Pa)" );
        Serial.println( "altitude <value>    - Set altitude (m)" );
        Serial.println( "temp <value>        - Set ambient temperature (deg)" );
        Serial.println( "boost <value>       - Set boost pressure (kPa)" );
        Serial.println( "show                - Show all current values" );
        Serial.println( "=====================\n" );
    } else if ( cmd == "debug on" ) {
        debug_mode = true;
        Serial.println( "Debug mode: ON" );
    } else if ( cmd == "debug off" ) {
        debug_mode = false;
        Serial.println( "Debug mode: OFF" );
    } else if ( cmd.startsWith( "dpf " ) ) {
        dpf_pm_accum = cmd.substring( 4 ).toFloat();
        Serial.printf( "DPF PM Accum: %.2f g/L\n", dpf_pm_accum );
    } else if ( cmd.startsWith( "dpfgen " ) ) {
        dpf_pm_gen = cmd.substring( 7 ).toFloat();
        Serial.printf( "DPF PM Gen: %.2f g/L\n", dpf_pm_gen );
    } else if ( cmd.startsWith( "dpfdist " ) ) {
        dpf_reg_dist = cmd.substring( 8 ).toInt();
        Serial.printf( "DPF Regen Dist: %d km\n", dpf_reg_dist );
    } else if ( cmd.startsWith( "dpfcount " ) ) {
        dpf_reg_count = cmd.substring( 9 ).toInt();
        Serial.printf( "DPF Regen Count: %d\n", dpf_reg_count );
    } else if ( cmd.startsWith( "dpfstatus " ) ) {
        dpf_reg_status = cmd.substring( 10 ).toInt();
        Serial.printf( "DPF Regen Status: %d\n", dpf_reg_status );
    } else if ( cmd.startsWith( "water " ) ) {
        engine_coolant_temp = cmd.substring( 6 ).toFloat();
        Serial.printf( "Water Temp: %.1f deg\n", engine_coolant_temp );
    } else if ( cmd.startsWith( "oil " ) ) {
        engine_oil_temp = cmd.substring( 4 ).toFloat();
        Serial.printf( "Oil Temp: %.1f deg\n", engine_oil_temp );
    } else if ( cmd.startsWith( "fuel " ) ) {
        fuel_level = cmd.substring( 5 ).toFloat();
        Serial.printf( "Fuel Level: %.1f %%\n", fuel_level );
    } else if ( cmd.startsWith( "pressure " ) ) {
        pressure = cmd.substring( 9 ).toFloat();
        Serial.printf( "Pressure: %.1f Pa (%.1f hPa)\n", pressure, pressure / 100.0 );
    } else if ( cmd.startsWith( "altitude " ) ) {
        altitude = cmd.substring( 9 ).toFloat();
        Serial.printf( "Altitude: %.1f m\n", altitude );
    } else if ( cmd.startsWith( "temp " ) ) {
        temp = cmd.substring( 5 ).toFloat();
        Serial.printf( "Ambient Temp: %.1f deg\n", temp );
    } else if ( cmd.startsWith( "boost " ) ) {
        boost_pressure = cmd.substring( 6 ).toFloat();
        Serial.printf( "Boost Pressure: %.1f kPa\n", boost_pressure );
    } else if ( cmd == "show" ) {
        Serial.println( "\n=== Current Values ===" );
        Serial.printf( "Debug Mode: %s\n", debug_mode ? "ON" : "OFF" );
        Serial.printf( "DPF PM Accum: %.2f g/L\n", dpf_pm_accum );
        Serial.printf( "DPF PM Gen: %.2f g/L\n", dpf_pm_gen );
        Serial.printf( "DPF Regen Dist: %d km\n", dpf_reg_dist );
        Serial.printf( "DPF Regen Count: %d\n", dpf_reg_count );
        Serial.printf( "DPF Regen Status: %d\n", dpf_reg_status );
        Serial.printf( "Water Temp: %.1f deg\n", engine_coolant_temp );
        Serial.printf( "Oil Temp: %.1f deg\n", engine_oil_temp );
        Serial.printf( "Fuel Level: %.1f %%\n", fuel_level );
        Serial.printf( "Pressure: %.1f hPa\n", pressure / 100.0 );
        Serial.printf( "Altitude: %.1f m\n", altitude );
        Serial.printf( "Ambient Temp: %.1f deg\n", temp );
        Serial.printf( "Boost Pressure: %.1f kPa\n", boost_pressure );
        Serial.println( "=====================\n" );
    } else if ( cmd.length() > 0 ) {
        Serial.println( "Unknown command. Type 'help' for available commands." );
    }
}

static void btn_process( int button, void func( void ) ) {
    Button* btn;
    switch ( button ) {
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
    if ( btn->wasPressed() ) {
        func();
    }
    // long press increase/decrease
    if ( btn->pressedFor( 750 ) && tmr_button[button] + 10 < millis() ) {
        tmr_button[button] = millis();
        func();
    }
}

// 進捗バー描画ヘルパー
static void drawProgressBar( int x, int y, int w, int h, float val, float maxVal, uint16_t color ) {
    int per = (int)( val / maxVal * 100 );
    per = min( per, 100 );
    sprite.drawRect( x, y, w, h, color );
    sprite.fillRect( x, y, w * per / 100, h, color );
}

// ゲージ描画ヘルパー（カラー表示対応）
// 温度に応じて色を変える（黒背景用に彩度を下げた色）
static uint16_t getTempColor( float temp, float lowThreshold, float midThreshold, float highThreshold ) {
    if ( temp <= 40 ) {
        // 青色（やや白寄り）- RGB565: 水色に近い青
        return 0x3D9F; // R=7, G=27, B=31 -> 薄い青
    } else if ( temp <= 80 ) {
        // 緑色（やや白寄り）- RGB565: 明るい緑
        return 0x87F0; // R=16, G=63, B=16 -> 薄い緑
    } else {
        // 赤色（やや白寄り）- RGB565: ピンクがかった赤
        return 0xFAAA; // R=31, G=21, B=10 -> 薄い赤
    }
}

// カラーゲージ描画（左右対称の半円ゲージ）
static void drawColorGauge( int centerX, int centerY, int radius, float val, float minVal, float maxVal,
                            float lowThreshold, float midThreshold, float highThreshold ) {
    // 背景の円弧（グレー）
    for ( int r = radius - 10; r <= radius; r++ ) {
        sprite.drawCircle( centerX, centerY, r, DARKGREY );
    }

    // 角度の計算（180度から0度まで、左から右へ）
    float ratio = ( val - minVal ) / ( maxVal - minVal );
    ratio = constrain( ratio, 0.0, 1.0 );
    int angle = 180 - (int)( ratio * 180 ); // 180度(左端)から0度(右端)まで

    // 色の決定
    uint16_t color = getTempColor( val, lowThreshold, midThreshold, highThreshold );

    // ゲージ部分の描画（180度から現在角度まで）
    for ( int a = 180; a >= angle; a -= 2 ) {
        float rad = a * PI / 180.0;
        int x1 = centerX + ( radius - 10 ) * cos( rad );
        int y1 = centerY - ( radius - 10 ) * sin( rad );
        int x2 = centerX + radius * cos( rad );
        int y2 = centerY - radius * sin( rad );
        sprite.drawLine( x1, y1, x2, y2, color );
    }
}

// 水温・油温共有ゲージ描画（270度から下を通って90度まで）
// 左半分（270度→180度→90度）：水温、右半分（270度→0度→90度）：油温
static void drawTempGauge( int centerX, int centerY, int radius, float waterTemp, float oilTemp ) {
    // 背景の円を描画（グレー） - 270度から時計回りに360度（0度経由で90度まで）
    for ( int r = radius - 10; r <= radius; r++ ) {
        // 270度 → 0度（右下半円）
        for ( int a = 270; a <= 360; a += 2 ) {
            float rad = a * PI / 180.0;
            int x = centerX + r * cos( rad );
            int y = centerY - r * sin( rad );
            sprite.drawPixel( x, y, DARKGREY );
        }
        // 0度 → 90度（右上）
        for ( int a = 0; a <= 90; a += 2 ) {
            float rad = a * PI / 180.0;
            int x = centerX + r * cos( rad );
            int y = centerY - r * sin( rad );
            sprite.drawPixel( x, y, DARKGREY );
        }
        // 90度 → 180度（左上）
        for ( int a = 90; a <= 180; a += 2 ) {
            float rad = a * PI / 180.0;
            int x = centerX + r * cos( rad );
            int y = centerY - r * sin( rad );
            sprite.drawPixel( x, y, DARKGREY );
        }
        // 180度 → 270度（左下）
        for ( int a = 180; a <= 270; a += 2 ) {
            float rad = a * PI / 180.0;
            int x = centerX + r * cos( rad );
            int y = centerY - r * sin( rad );
            sprite.drawPixel( x, y, DARKGREY );
        }
    }

    // 水温ゲージ（左半分：270度→180度→90度）
    float waterRatio = constrain( waterTemp / 120.0, 0.0, 1.0 );
    float waterAngleDeg = waterRatio * 180.0; // 0-180度の範囲
    uint16_t waterColor = getTempColor( waterTemp, 0, 0, 0 );

    // 270度から開始して反時計回りに進む
    for ( int a = 270; a >= 90; a -= 2 ) {
        if ( 270 - a <= waterAngleDeg ) {
            float rad = a * PI / 180.0;
            int x1 = centerX + ( radius - 10 ) * cos( rad );
            int y1 = centerY - ( radius - 10 ) * sin( rad );
            int x2 = centerX + radius * cos( rad );
            int y2 = centerY - radius * sin( rad );
            sprite.drawLine( x1, y1, x2, y2, waterColor );
        }
    }

    // 油温ゲージ（右半分：270度→0度→90度）
    float oilRatio = constrain( oilTemp / 140.0, 0.0, 1.0 );
    float oilAngleDeg = oilRatio * 180.0; // 0-180度の範囲
    uint16_t oilColor = getTempColor( oilTemp, 0, 0, 0 );

    // 270度から開始して時計回りに進む（270→360→0→90）
    float currentAngle = 0;
    for ( int a = 270; currentAngle <= oilAngleDeg && currentAngle <= 180; a += 2 ) {
        int actualAngle = a > 360 ? a - 360 : a;
        if ( actualAngle == 450 )
            actualAngle = 90; // 終点

        float rad = actualAngle * PI / 180.0;
        int x1 = centerX + ( radius - 10 ) * cos( rad );
        int y1 = centerY - ( radius - 10 ) * sin( rad );
        int x2 = centerX + radius * cos( rad );
        int y2 = centerY - radius * sin( rad );
        sprite.drawLine( x1, y1, x2, y2, oilColor );

        currentAngle += 2;
        if ( a >= 360 )
            break;
    }
    // 0度から90度まで継続
    if ( oilAngleDeg > 90 ) {
        for ( int a = 0; a <= 90 && a <= ( oilAngleDeg - 90 ); a += 2 ) {
            float rad = a * PI / 180.0;
            int x1 = centerX + ( radius - 10 ) * cos( rad );
            int y1 = centerY - ( radius - 10 ) * sin( rad );
            int x2 = centerX + radius * cos( rad );
            int y2 = centerY - radius * sin( rad );
            sprite.drawLine( x1, y1, x2, y2, oilColor );
        }
    }

    // 円の外側にラベル表示
    sprite.setFreeFont( &FreeMono12pt7b );
    int fontH = sprite.fontHeight();

    // Water（左側）
    sprite.setTextColor( waterColor );
    sprite.setCursor( centerX - radius - 68, centerY - 15 );
    sprite.printf( "Water" );
    // 水温（左側）
    char wbuf[8];
    snprintf( wbuf, sizeof( wbuf ), "%d", (int)waterTemp );
    int wWidth = sprite.textWidth( wbuf );
    int waterX = centerX - radius - 25 - ( wWidth / 2 );
    int textY = centerY + ( fontH / 2 ) + 15;
    sprite.setCursor( waterX, textY );
    sprite.printf( "%s", wbuf );

    // Oil（右側）
    sprite.setTextColor( oilColor );
    sprite.setCursor( centerX + radius + 18, centerY - 15 );
    sprite.printf( "Oil" );
    // 油温（右側）
    char obuf[8];
    snprintf( obuf, sizeof( obuf ), "%d", (int)oilTemp );
    int oWidth = sprite.textWidth( obuf );
    int oilX = centerX + radius + 25 - ( oWidth / 2 );
    sprite.setCursor( oilX, textY );
    sprite.printf( "%s", obuf );
}

/* ステータス盛り盛り表示
 * ・煤の堆積量
 * ・DPF再生からのTrip距離
 * ・水温
 * ・油温
 * ・燃料量
 * ・高度
 * ・地点気圧
 * ・海面気圧
 */
static void display_overview() {
    int percent = 0;
    if ( tmr + WAIT < millis() ) {
        tmr = millis();

        // display
        sprite.fillScreen( BLACK );
        SCREEN_TITLE( "Overview" );

        // 煤の堆積量（DPF進捗バーを短く、数字を大きく）
        uint16_t dpf_color = dpf_reg_status ? RED : WHITE;
        sprite.setTextColor( dpf_color );
        SET_FONT_AND_SIZE( FreeMono9pt7b, 1 );
        sprite.setCursor( 10, 35 );
        sprite.printf( "DPF" );
        float soot = max( dpf_pm_accum, dpf_pm_gen );
        drawProgressBar( 55, 25, 100, 15, soot, PM_MAX, dpf_color );
        // 数字を大きく表示（右合わせ）
        SET_FONT_AND_SIZE( FreeMonoBold18pt7b, 1 );
        char sootBuf[16];
        snprintf( sootBuf, sizeof( sootBuf ), "%.2f", soot );
        int sootWidth = sprite.textWidth( sootBuf );
        sprite.setCursor( 270 - sootWidth, 45 );
        sprite.printf( "%s", sootBuf );
        SET_FONT_AND_SIZE( FreeMono9pt7b, 1 );
        sprite.setCursor( 280, 35 );
        sprite.printf( "g/L" );

        // DPF再生からのTrip距離
        sprite.setTextColor( WHITE );
        SET_FONT_AND_SIZE( FreeMono9pt7b, 1 );
        sprite.setCursor( 10, 55 );
        sprite.printf( "Trip %dkm", dpf_reg_dist );

        // 高度
        sprite.setTextColor( WHITE );
        SET_FONT_AND_SIZE( FreeMono9pt7b, 1 );
        sprite.setCursor( 10, 80 );
        sprite.printf( "Altitude" );
        SET_FONT_AND_SIZE( FreeMonoBold18pt7b, 1 );
        char altBuf[16];
        snprintf( altBuf, sizeof( altBuf ), "%d", (int)gps.altitude.meters() );
        int altWidth = sprite.textWidth( altBuf );
        sprite.setCursor( 270 - altWidth, 80 );
        sprite.printf( "%s", altBuf );
        SET_FONT_AND_SIZE( FreeMono9pt7b, 1 );
        sprite.setCursor( 280, 80 );
        sprite.printf( "m" );

        // 燃料量（数値大きく、右寄せ）
        sprite.setTextColor( WHITE );
        SET_FONT_AND_SIZE( FreeMono9pt7b, 1 );
        sprite.setCursor( 10, 105 );
        sprite.printf( "Fuel" );
        SET_FONT_AND_SIZE( FreeMonoBold18pt7b, 1 );
        char fuelBuf[16];
        snprintf( fuelBuf, sizeof( fuelBuf ), "%d", (int)( fuel_level * FUEL_TANK_CAPACITY ) / 100 );
        int fuelWidth = sprite.textWidth( fuelBuf );
        sprite.setCursor( 270 - fuelWidth, 105 );
        sprite.printf( "%s", fuelBuf );
        SET_FONT_AND_SIZE( FreeMono9pt7b, 1 );
        sprite.setCursor( 280, 105 );
        sprite.printf( "L" );

        // 気圧（数値大きく、右寄せ）
        sprite.setTextColor( WHITE );
        SET_FONT_AND_SIZE( FreeMono9pt7b, 1 );
        sprite.setCursor( 10, 130 );
        sprite.printf( "Pressure" );
        SET_FONT_AND_SIZE( FreeMonoBold18pt7b, 1 );
        char pressBuf[16];
        snprintf( pressBuf, sizeof( pressBuf ), "%.1f", ( pressure / 100.0 ) );
        int pressWidth = sprite.textWidth( pressBuf );
        sprite.setCursor( 270 - pressWidth, 130 );
        sprite.printf( "%s", pressBuf );
        SET_FONT_AND_SIZE( FreeMono9pt7b, 1 );
        sprite.setCursor( 280, 130 );
        sprite.printf( "hPa" );

        // 水温・油温共有ゲージ（270度→0度→90度）
        drawTempGauge( 160, 190, 40, engine_coolant_temp, engine_oil_temp );

        // GPS情報
        uint32_t gps_color = gps.location.isValid() ? GREEN : RED;
        sprite.fillCircle( 300, 220, 8, gps_color );

        sprite.pushSprite( 0, 0 );
    }
}

static void display_altitude() {
    if ( tmr + WAIT < millis() ) {
        tmr = millis();

        // display
        sprite.fillScreen( BLACK );
        SCREEN_TITLE( "Altitude" );

        sprite.setTextColor( WHITE );
        SET_FONT_AND_SIZE( FreeMonoBold12pt7b, 2 );
        sprite.setCursor( 10, 110 );
        sprite.printf( "%4.2fm", altitude );

        SET_FONT_AND_SIZE( FreeMono9pt7b, 1 );
        sprite.setCursor( 20, 190 );
        sprite.printf( "sea:%.2fhPa", SEALEVELPRESSURE_HPA + sealevel_pressure_offset );
        sprite.setCursor( 20, 210 );
        sprite.printf( "cur:%.2fhPa", ( pressure / 100.0 ) );
        sprite.setCursor( 20, 230 );
        sprite.printf( "temp:%.2fdeg", ( temp ) );

        sprite.pushSprite( 0, 0 );
    }
}

static void display_dpf_status() {
    int percent = 0;
    if ( tmr + WAIT < millis() ) {
        tmr = millis();

        sprite.fillScreen( BLACK );
        if ( dpf_reg_status ) {
            SCREEN_TITLE_BG_RED( "DPF status" );
        } else {
            SCREEN_TITLE( "DPF status" );
        }

        sprite.setTextColor( WHITE );
        SET_FONT_AND_SIZE( FreeMono9pt7b, 1 );
        sprite.setCursor( 10, 75 );
        sprite.printf( "Acum" );
        sprite.setCursor( 230, 75 );
        sprite.printf( "%.2fg/L", dpf_pm_accum );
        drawProgressBar( 70, 60, 150, 30, dpf_pm_accum, PM_MAX, WHITE );

        sprite.setCursor( 10, 115 );
        sprite.printf( "Gene" );
        sprite.setCursor( 230, 115 );
        sprite.printf( "%.2fg/L", dpf_pm_gen );
        drawProgressBar( 70, 100, 150, 30, dpf_pm_gen, PM_MAX, WHITE );

        SET_FONT_AND_SIZE( FreeMono9pt7b, 2 );
        sprite.setCursor( 10, 170 );
        sprite.printf( "Count" );
        SET_FONT_AND_SIZE( FreeMono9pt7b, 3 );
        sprite.setCursor( 10, 220 );
        sprite.printf( "%d", dpf_reg_count );

        SET_FONT_AND_SIZE( FreeMono9pt7b, 2 );
        sprite.setCursor( 140, 170 );
        sprite.printf( "Dist" );
        SET_FONT_AND_SIZE( FreeMono9pt7b, 3 );
        sprite.setCursor( 140, 220 );
        sprite.printf( "%dkm", dpf_reg_dist );

        sprite.pushSprite( 0, 0 );
    }
}

static void display_additional_meter() {
    int percent = 0;
    if ( tmr + WAIT < millis() ) {
        tmr = millis();

        // display
        sprite.fillScreen( BLACK );
        SCREEN_TITLE( "Meter" );

        sprite.setTextColor( WHITE );
        SET_FONT_AND_SIZE( FreeMonoBold9pt7b, 1 );
        sprite.setCursor( 10, 60 );
        if ( dpf_reg_status ) {
            sprite.setTextColor( RED );
        }
        sprite.printf( "DPF" );

        sprite.setTextColor( WHITE );
        SET_FONT_AND_SIZE( FreeMono9pt7b, 1 );
        sprite.setCursor( 15, 80 );
        sprite.printf( "Accum" );
        drawProgressBar( 75, 70, 90, 15, dpf_pm_accum, PM_MAX, WHITE );

        sprite.setCursor( 15, 100 );
        sprite.printf( "Gen" );
        drawProgressBar( 75, 90, 90, 15, dpf_pm_gen, PM_MAX, WHITE );

        SET_FONT_AND_SIZE( FreeMono9pt7b, 2 );
        sprite.setCursor( 200, 70 );
        sprite.printf( "Dist" );
        sprite.setCursor( 200, 100 );
        sprite.printf( "%dkm", dpf_reg_dist );

        SET_FONT_AND_SIZE( FreeMonoBold9pt7b, 1 );
        sprite.setCursor( 10, 140 );
        sprite.printf( "Temperature" );

        SET_FONT_AND_SIZE( FreeMono9pt7b, 2 );
        sprite.setCursor( 15, 170 );
        sprite.printf( "Wtr" );
        sprite.setCursor( 100, 170 );
        sprite.printf( "%d", (int)engine_coolant_temp );

        sprite.setCursor( 15, 200 );
        sprite.printf( "Oil" );
        sprite.setCursor( 100, 200 );
        sprite.printf( "%d", (int)engine_oil_temp );

        sprite.setCursor( 180, 150 );
        sprite.printf( "Boost" );
        sprite.setCursor( 180, 180 );
        sprite.printf( "%dkpa", (int)boost_pressure );

        sprite.pushSprite( 0, 0 );
    }
}

static void display_setting() {
    if ( tmr + WAIT < millis() ) {
        tmr = millis();

        sprite.fillScreen( BLACK );
        SCREEN_TITLE( "Setting" );

        SET_FONT_AND_SIZE( FreeMono9pt7b, 1 );
        sprite.setCursor( 10, 60 );
        sprite.setTextColor( WHITE );
        sprite.printf( "Altitude temp from: " );
        sprite.printf( "%s", is_temperature_from_sensord ? "Sensord" : "OBD2" );

        sprite.setCursor( 10, 90 );
        sprite.setTextColor( WHITE );
        sprite.printf( "Ambient temp: " );
        sprite.printf( "%2.2f", car_outside_temperature );

        sprite.setCursor( 10, 200 );
        sprite.printf( "A:sensor" );
        sprite.setCursor( 10, 220 );
        sprite.printf( "C:Clear" );
    }

    sprite.pushSprite( 0, 0 );
}

/***********************************/
/* Global functions                */
/***********************************/
void setup() {
    bool error = false;

    M5.begin();
    M5.Speaker.begin();
    M5.Speaker.mute();
    M5.Lcd.setRotation( 1 );
    preferences.begin( "myApp", false );
    screen = preferences.getInt( "screen", SCREEN_OVERVIEW );
    is_temperature_from_sensord = preferences.getBool( "sensor_type", true );

    sprite.setColorDepth( 8 );
    sprite.setTextSize( 2 );
    sprite.createSprite( M5.Lcd.width(), M5.Lcd.height() );
    sprite.fillScreen( BLACK );

    // Initializing Serial
    Serial.begin( 115200 );
    while ( !Serial )
        ;

    Serial.println( "\n\n=== CarMonitor Debug Mode ===" );
    Serial.println( "Type 'help' for available commands" );
    Serial.println( "Type 'debug on' to enable debug mode" );
    Serial.println( "============================\n" );

    // Initializing BMP280
    bmp_init();

    // Initializing ELM327
    car_param_init();

    // Initializing GPS
    Serial.begin( 115200 );
    ss.begin( 9600 );

    delay( 1000 ); // wait for sensor init
}

void loop() {
    M5.update();

    // デバッグモード: シリアルコマンド受信処理
    while ( Serial.available() > 0 ) {
        char c = Serial.read();
        if ( c == '\n' || c == '\r' ) {
            if ( serial_command.length() > 0 ) {
                process_debug_command( serial_command );
                serial_command = "";
                Serial.print( "> " ); // プロンプト表示
            }
        } else {
            serial_command += c;
        }
    }

    // デバッグモードでない場合は通常のOBD2/センサー読み取りを実行
    if ( !debug_mode ) {
        car_param_exec();
    }

    switch ( screen ) {
    case SCREEN_OVERVIEW:
        display_overview();
        btn_process( BTN_A, []() { sealevel_pressure_offset += 0.5; } );
        btn_process( BTN_C, []() { sealevel_pressure_offset += -0.5; } );
        break;
    // case SCREEN_ALTITUDE:
    //     display_altitude();
    //     btn_process( BTN_A, []() { sealevel_pressure_offset += 0.5; } );
    //     btn_process( BTN_C, []() { sealevel_pressure_offset += -0.5; } );
    //     break;
    case SCREEN_DPF_STATUS:
        display_dpf_status();
        break;
    case SCREEN_ADDITIONAL_METER:
        display_additional_meter();
        break;
    case SCREEN_SETTING:
        display_setting();
        btn_process( BTN_A, []() {
            is_temperature_from_sensord = !is_temperature_from_sensord;
            preferences.putBool( "sensor_type", is_temperature_from_sensord );
        } );
        btn_process( BTN_C, []() {
            // preferenceをクリアしたらソフトリセットする
            preferences.clear();
            ESP.restart();
        } );
        break;
    default:
        break;
    }

    btn_process( BTN_B, []() {
        screen++;
        if ( screen >= SCREEN_NUM ) {
            screen = SCREEN_OVERVIEW;
        }
        preferences.putInt( "screen", screen );
    } );

    if ( tmr_save_prefarences + SAVE_PREFARENCES_CYCLE < millis() ) {
        tmr_save_prefarences = millis();
        if ( (int)altitude_old != (int)altitude ) { // To reduce the number of write acceses to flash memory
            altitude_old = altitude;
            preferences.putFloat( "altitude", altitude );
        }
    }

    while ( ss.available() > 0 ) {
        char c = ss.read();
        gps.encode( c );
    }
}
