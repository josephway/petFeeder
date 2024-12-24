/*
 * 2024年上海市少年儿童乐创挑战系列活动
 *
 * 参赛队名：智慧星河队
 * 学校：上海市静安区一中心小学
 * 班级：三（6）班
 * 队员：王一野、乔安易、程云骞、夏长榎
 * 指导老师：陈晓婧  丁毅
 * 报名编号：TFJ10037
 *
 * 智能宠物喂食器
 *
 * 功能概述：
 * 1. 自动喂食
 *    - 定时喂食：每8小时自动投放一次
 *    - 手动喂食：通过按钮触发或远程IoT控制
 *    - 可调节份量：支持小份、中份、大份
 *    - 食盘防溢出：实时监测食盘重量，超过阈值自动停止投食
 *
 * 2. 智能监测
 *    - 储粮监测：通过累计投放量计算剩余量，及时提醒补充
 *    - 宠物监测：通过超声波传感器检测宠物是否正常进食
 *    - 在线状态：支持WiFi连接，可远程监控设备状态
 *
 * 3. 按键操作
 *    - P0按键单击：执行喂食
 *    - P0按键长按：进入称重校准模式
 *    - ButtonA：切换份量（小份/中份/大份）
 *    - ButtonB：重置储粮量
 *
 * 4. 网络功能
 *    - 远程控制：支持远程触发喂食和调整份量
 *    - 状态上报：定期上报设备运行状态
 *    - 断线重连：支持自动重连和离线模式切换
 *
 * 硬件要求：
 * - 掌控板(支持Arduino IDE和Mind+编程环境)
 * - 舵机（投食机构）
 * - DFRobot_HX711_I2C称重模块
 * - 超声波测距模块
 * - 蜂鸣器
 * - WiFi模块（内置）
 */

// 基础库
#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_Servo.h>
#include "DFRobot_HX711_I2C.h"
#include "MPython.h"
#include "DFRobot_Iot.h"

// IoT配置
#define IOT_SERVER "iot.dfrobot.com.cn" // IoT服务器地址
#define IOT_ID "Ygr7R3SNg"
#define IOT_PWD "YR97RqSHRz"
#define WIFI_SSID "你的WiFi名称"
#define WIFI_PASSWORD "你的WiFi密码"

// 定义IoT主题
#define TOPIC_FEED "_DjYg3IHR"    // 远程喂食指令
#define TOPIC_PORTION "OM56R3INg" // 远程调整份量
#define TOPIC_STATUS "Vpueg3INR"  // 状态上报

// 引脚定义 - 掌控板专用引脚命名
#define BUTTON_PIN P0  // 多功能按钮输入引脚（支持单击/双击/三击/长按）
#define BUZZER_PIN P16 // 蜂鸣器输出引脚（用于声音提示）
#define TRIG_PIN P2    // 超声波模块Trig引脚（发送信号）
#define ECHO_PIN P3    // 超声波模块Echo引脚（接收信号）
#define SERVO_PIN P15  // 舵机控制引脚（控制投食机构）

// I2C设备配置
#define HX711_I2C_ADDR 0x64 // DFRobot HX711称重模块I2C地址（使用默认地址）

// 重量相关阈值
#define MAX_FOOD_WEIGHT 100.0    // 食盆最大重量（克）
#define MIN_STORAGE_WEIGHT 100.0 // 储粮桶最小重量（克）
#define LOW_THRESHOLD 200.0      // 缺粮警告阈值（克）

// 喂食份量定义
#define SERVO_ANGLE 90 // 舵机转动角度

// 距离检测相关
#define MAX_DISTANCE 200      // 超声波最大测量距离(cm)
#define DISTANCE_THRESHOLD 20 // 宠物检测阈值距离(cm)
#define PET_ABSENCE_HOURS 12  // 宠物离开警告时间阈值（小时）

// 按钮相关常量
#define DEBOUNCE_TIME 50           // 按钮消抖时间(ms)
#define CLICK_TIMEOUT 500          // 点击超时时间(ms)
#define LONG_PRESS_TIME 2000       // 长按时间(ms)
#define INITIAL_FOOD_AMOUNT 1000.0 // 初始储粮量(g)

// 喂食间隔设置
#define FEEDING_INTERVAL 28800000 // 喂食间隔(8小时 = 8 * 60 * 60 * 1000 ms)
#define FEEDING_TIMES_PER_DAY 3   // 每天喂食次数

// 喂食相关时间定义
#define FEED_TIME_MIN 300 // 最小喂食时间(ms)，对应最小份量
#define FEED_TIME_MAX 800 // 最大喂食时间(ms)，对应最大份量

// 声音模式枚举
enum SoundPattern
{
    ALERT_FOOD_LOW,       // 储粮不足警告
    ALERT_BOWL_FULL,      // 食盆已满警告
    ALERT_PET_ABSENT,     // 宠物离开警告
    ALERT_HARDWARE_ERROR, // 硬件错误警告
    SOUND_FEED_SUCCESS,   // 喂食成功提示
    SOUND_PORTION_CHANGE  // 份量切换提示
};

// 创建对象
Servo feedServo;
DFRobot_HX711_I2C bowlScale(&Wire, HX711_I2C_ADDR);

// 份量相关
enum FoodPortion
{
    SMALL,
    MEDIUM,
    LARGE
};
FoodPortion currentPortion = MEDIUM;

// 按钮状态变量
float remainingFood = INITIAL_FOOD_AMOUNT; // 剩余储粮量

// 喂食计时
unsigned long lastFeedingTime = 0; // 上次喂食时间

// 创建IoT对象
DFRobot_Iot myIot;

// 网络状态管理
#define RECONNECT_INTERVAL 3600000 // 重连间隔(1小时)

// 网络状态变量
bool isOnline = false;
unsigned long lastConnectionAttempt = 0;

// 全局时间变量声明
unsigned long currentMillis = 0;         // 当前时间戳
unsigned long lastPetVisitTime = 0;      // 上次宠物访问时间
unsigned long lastVisitMillis = 0;       // 用于检测宠物离开
unsigned long lastApproachTime = 0;      // 上次接近时间
unsigned long lastConnectionAttempt = 0; // 上次WiFi连接尝试时间

// 其他全局变量
int approachCount = 0;           // 接近次数计数
int reconnectAttempts = 0;       // 重连尝试次数
#define APPROACH_COOLDOWN 600000 // 接近检测��却时间（10分钟）

// 连接函数
bool connectToIot()
{
    if (myIot.wifiConnect(WIFI_SSID, WIFI_PASSWORD))
    {
        displayMessage("WiFi连接", "连接IoT中...");
        delay(1000);

        if (myIot.init(IOT_SERVER, IOT_ID, "", IOT_PWD, TOPIC_FEED, TOPIC_PORTION, TOPIC_STATUS, 1883))
        {
            isOnline = true;
            displayMessage("网络已连接", "在线模式");
            return true;
        }
    }

    isOnline = false;
    displayMessage("网络未连接", "离线模式");
    return false;
}

// 网络管理函数
void manageConnection()
{
    if (!isOnline && millis() - lastConnectionAttempt >= RECONNECT_INTERVAL)
    {
        lastConnectionAttempt = millis();
        displayMessage("尝试重连中...");
        connectToIot();
    }
}

// 显示相关函数
void displayMessage(const char *line1, const char *line2 = "")
{
    display.fillScreen(0); // 清屏
    display.setCursor(0, 0);
    display.print(line1); // 第一行文字
    if (strlen(line2) > 0)
    {
        display.setCursor(0, 16);
        display.print(line2); // 二行文字
    }
    display.show(); // 显示
}

// 2. 核心功能函数
// 移动喂食相关函数到一起
void feed()
{
    float initialBowlWeight = bowlScale.readWeight();
    float portionWeight = getCurrentPortionWeight();

    if (initialBowlWeight >= MAX_FOOD_WEIGHT)
    {
        playSound(ALERT_BOWL_FULL);
        displayMessage("食盆已满", "暂停投食");
        return;
    }

    if (remainingFood <= MIN_STORAGE_WEIGHT)
    {
        playSound(ALERT_FOOD_LOW);
        displayMessage("储粮不足", "请及时补充");
        return;
    }

    // 执行喂食
    feedServo.write(SERVO_ANGLE);
    int feedTime = map(portionWeight, PORTION_SMALL, PORTION_LARGE, FEED_TIME_MIN, FEED_TIME_MAX);
    delay(feedTime);
    feedServo.write(0);

    // 等待食物完全落下
    delay(1000);

    // 测量实际投放重量
    float finalBowlWeight = bowlScale.readWeight();
    float actualFeedAmount = finalBowlWeight - initialBowlWeight;

    // 更新剩余储量
    if (actualFeedAmount > 0)
    {
        remainingFood -= actualFeedAmount;

        // 显示实际投放量
        char feedMsg[32];
        sprintf(feedMsg, "实际: %.1fg", actualFeedAmount);
        displayMessage("喂食完成", feedMsg);
    }
    else
    {
        // 如果没有检测到重量变化，可能是出现故障
        playSound(ALERT_HARDWARE_ERROR);
        displayMessage("喂食异常", "请检查设备");
    }

    delay(2000);
}

float getCurrentPortionWeight()
{
    switch (currentPortion)
    {
    case SMALL:
        return PORTION_SMALL;
    case MEDIUM:
        return PORTION_MEDIUM;
    case LARGE:
        return PORTION_LARGE;
    default:
        return PORTION_MEDIUM;
    }
}

void checkFeedingTime()
{
    unsigned long currentTime = millis();

    // 检查是否到达喂食时间
    if (currentTime - lastFeedingTime >= FEEDING_INTERVAL)
    {
        feed();
        lastFeedingTime = currentTime;

        // 显示喂食次数
        char countStr[32];
        sprintf(countStr, "第%lu次喂食", (currentTime - lastFeedingTime) / FEEDING_INTERVAL);
        displayMessage(countStr);
        delay(2000);
    }
}

void adjustPortion()
{
    currentPortion = (FoodPortion)((currentPortion + 1) % 3);

    char portionMsg[32];
    sprintf(portionMsg, "已切换为%s", getCurrentPortionText());
    displayMessage("份量调整", portionMsg);

    playSound(SOUND_PORTION_CHANGE);
    delay(1000);
}

const char *getCurrentPortionText()
{
    switch (currentPortion)
    {
    case SMALL:
        return "小份量";
    case MEDIUM:
        return "中份量";
    case LARGE:
        return "大份量";
    default:
        return "中份量";
    }
}

void resetFoodAmount()
{
    // 蜂鸣器提示音（三声短响）
    for (int i = 0; i < 3; i++)
    {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(100);
        digitalWrite(BUZZER_PIN, LOW);
        delay(100);
    }

    // 显示重置开始提示
    displayMessage("储粮量重置中", "请稍候...");
    delay(1000);

    remainingFood = INITIAL_FOOD_AMOUNT;

    // 显示重置完成信息
    char resetMsg[32];
    sprintf(resetMsg, "当前储量: %dg", (int)INITIAL_FOOD_AMOUNT);
    displayMessage("重置完成!", resetMsg);

    delay(2000);

    // 最后一声响表示完成
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
}

// 移动按钮处理相关函数到一起
void handleButton()
{
    static bool isLongPress = false;
    static unsigned long pressStartTime = 0;

    // 用掌控板内建的ButtonA和ButtonB
    if (digitalRead(BUTTON_PIN) == LOW) // 检测P0引脚单击
    {
        if (pressStartTime == 0)
        {
            pressStartTime = millis();
        }

        // 检测长按（用于校准）
        if (!isLongPress && (millis() - pressStartTime >= LONG_PRESS_TIME))
        {
            isLongPress = true;
            calibrateScale();
        }
    }
    else if (pressStartTime != 0) // 按钮被释放
    {
        unsigned long pressDuration = millis() - pressStartTime;

        // 如果不是长按，则处理单击
        if (!isLongPress && pressDuration < LONG_PRESS_TIME)
        {
            feed(); // 单击执行喂食
        }

        // 重置状态
        pressStartTime = 0;
        isLongPress = false;
    }

    if (buttonA.isPressed())
    {
        adjustPortion();
    }

    if (buttonB.isPressed())
    {
        resetFoodAmount();
    }
}

// 移动传感器相关函数到一起
void checkPetApproach()
{
    // 使用MPython库中的超声波测距函数
    float distance = ultrasonicDistance(TRIG_PIN, ECHO_PIN);

    // 如果距离在有效范围内且小于阈值，说明宠物接近
    if (distance > 0 && distance < DISTANCE_THRESHOLD)
    {
        if (millis() - lastApproachTime > APPROACH_COOLDOWN)
        {
            lastPetVisitTime = millis(); // 更新最后访问时间
            approachCount++;
            lastApproachTime = millis();

            // 显示检测到宠物
            displayMessage("检测到宠物",
                           String("距离: " + String(distance, 1) + "cm").c_str());
        }
    }
}

void checkPetAbsence()
{
    // 使用超声波检测宠物
    float distance = ultrasonicDistance(TRIG_PIN, ECHO_PIN);
    if (distance > 0 && distance < DISTANCE_THRESHOLD)
    {
        lastVisitMillis = millis();
    }

    if (millis() - lastVisitMillis > PET_ABSENCE_HOURS * 3600000)
    {
        playSound(ALERT_PET_ABSENT);
        unsigned long hoursAway = (millis() - lastVisitMillis) / 3600000;
        unsigned long minutesAway = ((millis() - lastVisitMillis) % 3600000) / 60000;

        char absenceStr[32];
        sprintf(absenceStr, "%lu小时%lu分", hoursAway, minutesAway);
        displayMessage("宠物已离开", absenceStr);
    }
}

void calibrateScale()
{
    displayMessage("校准称重", "请清空食盆");
    delay(3000);

    bowlScale.peel(); // 去皮

    displayMessage("放入标准重物", "100g");
    delay(5000);

    float calibrationWeight = 100.0f; // 标准重物重量
    float rawValue = bowlScale.readWeight();
    float calibrationValue = calibrationWeight / rawValue;

    bowlScale.setCalibration(calibrationValue);

    displayMessage("校准完成", "");
    delay(2000);
}

// 声音模式管理函数
void playSound(SoundPattern pattern)
{
    switch (pattern)
    {
    case ALERT_FOOD_LOW:
        // 储粮不足：三长
        for (int i = 0; i < 3; i++)
        {
            digitalWrite(BUZZER_PIN, HIGH);
            delay(500);
            digitalWrite(BUZZER_PIN, LOW);
            delay(200);
        }
        break;

    case ALERT_BOWL_FULL:
        // 食盆已满：两短一长
        for (int i = 0; i < 2; i++)
        {
            digitalWrite(BUZZER_PIN, HIGH);
            delay(100);
            digitalWrite(BUZZER_PIN, LOW);
            delay(100);
        }
        digitalWrite(BUZZER_PIN, HIGH);
        delay(500);
        digitalWrite(BUZZER_PIN, LOW);
        break;

    case ALERT_PET_ABSENT:
        // 宠物离开：四短音
        for (int i = 0; i < 4; i++)
        {
            digitalWrite(BUZZER_PIN, HIGH);
            delay(100);
            digitalWrite(BUZZER_PIN, LOW);
            delay(100);
        }
        break;

    case ALERT_HARDWARE_ERROR:
        // 硬件错误：一长两短
        digitalWrite(BUZZER_PIN, HIGH);
        delay(500);
        digitalWrite(BUZZER_PIN, LOW);
        delay(200);
        for (int i = 0; i < 2; i++)
        {
            digitalWrite(BUZZER_PIN, HIGH);
            delay(100);
            digitalWrite(BUZZER_PIN, LOW);
            delay(100);
        }
        break;

    case SOUND_FEED_SUCCESS:
        // 喂食成功：一短音
        digitalWrite(BUZZER_PIN, HIGH);
        delay(200);
        digitalWrite(BUZZER_PIN, LOW);
        break;

    case SOUND_PORTION_CHANGE:
        // 份量切换：两短音
        for (int i = 0; i < 2; i++)
        {
            digitalWrite(BUZZER_PIN, HIGH);
            delay(100);
            digitalWrite(BUZZER_PIN, LOW);
            delay(100);
        }
        break;
    }
}

// 修改显示函数以包含在线状态
void displayStatus()
{
    display.fillScreen(0); // 清屏

    // 第一行：时间和在线状态（限制长度避免溢出）
    char timeStr[32];
    snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu %s %s",
             (millis() / 3600000) % 24,
             (millis() / 60000) % 60,
             getCurrentPortionText(),
             isOnline ? "在线" : "离线");
    display.setCursor(0, 0);
    display.print(timeStr);

    // 第二行：储粮信息
    char statusStr[32];
    snprintf(statusStr, sizeof(statusStr), "储粮:%.1fg", remainingFood);
    display.setCursor(0, 16);
    display.print(statusStr);

    // 第三行：宠物状态
    char petStr[32];
    unsigned long minutesAgo = lastPetVisitTime > 0 ? (millis() - lastPetVisitTime) / 60000 : 0;
    snprintf(petStr, sizeof(petStr), "上次:%lu分钟前", minutesAgo);
    display.setCursor(0, 32);
    display.print(petStr);

    // 第四行：网络状态（仅在离线时显示重连信息）
    if (!isOnline && reconnectAttempts > 0)
    {
        char reconnectStr[32];
        snprintf(reconnectStr, sizeof(reconnectStr), "重连:%d次", reconnectAttempts);
        display.setCursor(0, 48);
        display.print(reconnectStr);
    }

    display.show(); // 更新显示
}

// IoT消息处理函数
void processIotMessages()
{
    if (!isOnline)
        return;

    String topic = myIot.topicRead();
    if (topic.length() > 0)
    {
        String message = myIot.messageRead();

        if (topic == TOPIC_FEED)
        {
            feed();
        }
        else if (topic == TOPIC_PORTION)
        {
            int portion = message.toInt();
            switch (portion)
            {
            case 1:
                currentPortion = SMALL;
                break;
            case 2:
                currentPortion = MEDIUM;
                break;
            case 3:
                currentPortion = LARGE;
                break;
            }
            adjustPortion();
        }
    }
}

// 状态上报函数
void reportStatus()
{
    if (!isOnline)
        return;

    String status = "{";
    status += "\"remaining_food\":" + String(remainingFood) + ",";
    status += "\"portion\":\"" + String(getCurrentPortionText()) + "\",";
    status += "\"bowl_weight\":" + String(bowlScale.readWeight()) + ",";
    status += "\"online\":true}";

    myIot.publish(TOPIC_STATUS, status);
}

// 添加错误显示函数（之前被引用但未定义）
void displayError(const char *error)
{
    display.fillScreen(0);
    display.setCursor(0, 0);
    display.print("错误:");
    display.setCursor(0, 16);
    display.print(error);
    display.show();
}

// 3. 主程序函数
void setup()
{
    Serial.begin(115200);
    Wire.begin();

    // 始化显示
    display.begin();       // 初始化显示
    display.fillScreen(0); // 清屏
    display.show();

    feedServo.attach(SERVO_PIN);
    feedServo.write(0);

    while (!bowlScale.begin())
    {
        playSound(ALERT_HARDWARE_ERROR);
        displayError("称重模块错误");
        delay(2000);
    }

    bowlScale.setCalibration(2.0f);
    bowlScale.peel();

    pinMode(BUTTON_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    displayMessage("宠物喂食器", "系统就绪");
    playSound(SOUND_FEED_SUCCESS);

    lastFeedingTime = millis();

    // 尝试连接Easy IoT
    connectToIot();

    // 初始化时间相关变量
    lastPetVisitTime = millis();
    lastVisitMillis = millis();
    lastApproachTime = millis();
    lastConnectionAttempt = millis();
}

void loop()
{
    // 1. 更新系统状态
    handleButton(); // 调用按钮处理函数
    unsigned long runTime = millis();

    // 网络管理
    manageConnection();

    // 仅在在线状态下处理IoT消息
    if (isOnline)
    {
        processIotMessages();

        // 状态上报
        static unsigned long lastReport = 0;
        if (millis() - lastReport >= 60000)
        {
            reportStatus();
            lastReport = millis();
        }
    }

    // 更新显示
    displayStatus();

    // 2. 显示信息
    checkFeedingTime();
    checkPetApproach();
    checkPetAbsence();

    delay(100);
}
