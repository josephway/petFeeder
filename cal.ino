#include <Wire.h>
#include "DFRobot_HX711_I2C.h"

// I2C地址
#define HX711_I2C_ADDR 0x64

// 创建称重模块对象
DFRobot_HX711_I2C scale(&Wire, HX711_I2C_ADDR);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // 初始化称重模块
    while (!scale.begin()) {
        Serial.println("称重模块初始化失败，请检查连接。");
        delay(2000);
    }

    // 去皮
    scale.peel();
    delay(2000); // 增加延迟以确保去皮完成
    Serial.println("请放置已知重量的物体（257克）...");
    delay(5000); // 等待用户放置物体

    // 读取原始数据
    float rawWeight = scale.readWeight();
    Serial.print("读取的原始重量数据: ");
    Serial.println(rawWeight);

    // 计算校准因子
    float calibrationFactor = rawWeight / 257.0;
    Serial.print("计算得到的校准因子: ");
    Serial.println(calibrationFactor);

    // 提示用户移除物体
    Serial.println("请移除物体。");
}

void loop() {
    // 空循环
}