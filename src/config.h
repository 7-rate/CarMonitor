#pragma once

// 本システムのコンフィグ設定を行うためのファイルです

// BMP280(気圧/温度センサー)の有無
#define CONFIG_HAS_BMP280 ( 1 )

// MPU6050(加速度/ジャイロセンサー)の有無
#define CONFIG_HAS_MPU6050 ( 1 )

// 画面の有効無効
// 気圧高度表示
#define CONFIG_ENABLE_SCREEN_ALTITUDE ( 0 )
// IMUデバッグ表示
#define CONFIG_ENABLE_SCREEN_YRP ( 0 )
// IMUによる姿勢表示
#define CONFIG_ENABLE_SCREEN_ACC_MONITOR ( 0 )
