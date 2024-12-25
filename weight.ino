#include <Wire.h>
#include "DFRobot_HX711_I2C.h"


// 创建称重模块对象
DFRobot_HX711_I2C scale;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // 初始化称重模块
    while (!scale.begin()) {
        Serial.println("称重模块初始化失败，请检查连接。");
        delay(2000);
    }

    // 设置固定的校准因子
    scale.setCalibration(1677.f);
    scale.peel();
    delay(5000); // 增加去皮时间

    Serial.println("开始称重...");
}

void loop() {
    // 读取多次重量并取平均值
    float totalWeight = 0;
    int readings = 10;
    for (int i = 0; i < readings; i++) {
        totalWeight += scale.readWeight();
        delay(100); // 每次读取间隔
    }
    float averageWeight = totalWeight / readings;

    Serial.print("当前平均重量: ");
    Serial.print(averageWeight);
    Serial.println(" 克");

    // 每隔一秒更新一次
    delay(1000);
}
