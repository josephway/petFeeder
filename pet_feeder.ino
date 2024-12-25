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
#include <DFRobot_URM10.h>

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
#define BUTTON_PIN P8  // 多功能按钮输入引脚（支持单击/双击/三击/长按）
#define SERVO_PIN P0  // 舵机控制引脚（控制投食机构）
#define LED_PIN P9 // 定义LED灯连接的引脚


// 重量相关阈值
#define MAX_FOOD_WEIGHT 100.0    // 最大重量（克）
#define MIN_STORAGE_WEIGHT 100.0 // 储粮桶最小重量（克）
#define LOW_THRESHOLD 200.0      // 缺粮警告阈值（克）
#define PORTION_SMALL 10.0  // 小份量重量（克）
#define PORTION_MEDIUM 20.0 // 中份量重量（克）
#define PORTION_LARGE 30.0  // 大份量重量（克）


// 喂食份量定义
#define SERVO_ANGLE 90 // 舵机转动角度

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

const String topics[5] = {TOPIC_FEED,TOPIC_PORTION,TOPIC_STATUS,"",""}; // 定义IoT主题

// 函数声明
void displayMessage(const char *line1, const char *line2 = "");
void playSound(SoundPattern pattern);
float getCurrentPortionWeight();
const char *getCurrentPortionText();
void feed();
void adjustPortion();
void resetFoodAmount();
void handleButton();
void checkPetApproach();
void checkPetAbsence();
void calibrateScale();
void displayStatus();
void processIotMessages();
void reportStatus();
void displayError(const char *error);
bool connectToIot();
void manageConnection();
void checkFeedingTime();

// 创建对象
Servo feedServo;
DFRobot_HX711_I2C bowlScale;

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
unsigned long lastVisitMillis = 0;       // 用于检测���物离开
unsigned long lastApproachTime = 0;      // 上次接近时间

// 其他全局变量
int approachCount = 0;           // 接近次数计数
int reconnectAttempts = 0;       // 重连尝试次数
#define APPROACH_COOLDOWN 600000 // 接近检测却时间（10分钟）

DFRobot_URM10 urm10; // 在全局范围内声明

// 连接函数
bool connectToIot()
{
    myIot.wifiConnect(WIFI_SSID, WIFI_PASSWORD); // 调用连接函数

    // 使用 wifiStatus() 检查连接状态
    if (myIot.wifiStatus()) // 假设 wifiStatus() 返回 true 表示连接成功
    {
        displayMessage("WiFi连接", "连接IoT中...");
        delay(1000);

        myIot.init(IOT_SERVER, IOT_ID, "8282481049894439", IOT_PWD, topics, 1883);
        
        if (myIot.connected()) // 使用 myIot.connected() 判断连接状态
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
void displayMessage(const char *line1, const char *line2)
{
    display.fillScreen(0); // 清屏
    display.setCursor(0, 0);
    display.print(line1); // 第一行文字
    if (strlen(line2) > 0)
    {
        display.setCursor(0, 16);
        display.printLine(line2); // 第二行文字
    }
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

    // 点亮LED灯
    digitalWrite(LED_PIN, HIGH);

    // 执行喂食
    feedServo.angle(SERVO_ANGLE);
    int feedTime = map(portionWeight, PORTION_SMALL, PORTION_LARGE, FEED_TIME_MIN, FEED_TIME_MAX);
    delay(feedTime);
    feedServo.angle(0);

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
        // // 如果没有检测到重量变化，可能是出现故障
        // playSound(ALERT_HARDWARE_ERROR);
        // displayMessage("喂食异常", "请检查设备");
    }

    // 关闭LED灯
    digitalWrite(LED_PIN, LOW);

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
    // 使用内置蜂鸣器播放三声短响

    buzz.freq(784, BEAT_1);
    delay(200);

    // 显示重置开始提示
    displayMessage("储粮量重置中", "请稍候...");
    delay(1000);

    remainingFood = INITIAL_FOOD_AMOUNT;

    // 显示重置完成信息
    char resetMsg[32];
    sprintf(resetMsg, "当前储量: %dg", (int)INITIAL_FOOD_AMOUNT);
    displayMessage("重置完成!", resetMsg);

    delay(1000);
}

// 移动按钮处理相关函数到一起
void handleButton()
{
    static bool isLongPress = false;
    static unsigned long pressStartTime = 0;

    // 检测P8引脚单击（按钮按下高电平）
    if (digitalRead(BUTTON_PIN) == HIGH) // 修改为检测高电平
    {
        if (pressStartTime == 0)
        {
            pressStartTime = millis();
        }

        // 检测长按（用于校准）
        if (!isLongPress && (millis() - pressStartTime >= LONG_PRESS_TIME))
        {
            isLongPress = true;
            // calibrateScale(); // 注释掉校准功能调用
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

// 注释掉与测距相关的函数调用和实现
void checkPetApproach()
{
    // float distance = urm10.getDistanceCM(TRIG_PIN, ECHO_PIN); // 使用 getDistanceCM() 方法获距离

    // 如果距离在有效范围内且小于阈值，说明宠物接近
    // if (distance > 0 && distance < DISTANCE_THRESHOLD)
    // {
    //     if (millis() - lastApproachTime > APPROACH_COOLDOWN)
    //     {
    //         lastPetVisitTime = millis(); // 更新最后访问时间
    //         approachCount++;
    //         lastApproachTime = millis();

    //         // 显示检测到宠物
    //         displayMessage("检测到宠物",
    //                        String("距离: " + String(distance, 1) + "cm").c_str());
    //     }
    // }
}

void checkPetAbsence()
{
    // 使用超声波检测宠物
    // float distance = urm10.getDistanceCM(TRIG_PIN, ECHO_PIN); // 修改为使用 getDistanceCM() 方法
    // if (distance > 0 && distance < DISTANCE_THRESHOLD)
    // {
    //     lastVisitMillis = millis();
    // }

    // if (millis() - lastVisitMillis > PET_ABSENCE_HOURS * 3600000)
    // {
    //     playSound(ALERT_PET_ABSENT);
    //     unsigned long hoursAway = (millis() - lastVisitMillis) / 3600000;
    //     unsigned long minutesAway = ((millis() - lastVisitMillis) % 3600000) / 60000;

    //     char absenceStr[32];
    //     sprintf(absenceStr, "%lu小时%lu分", hoursAway, minutesAway);
    //     displayMessage("宠物已离开", absenceStr);
    // }
}

// 声音模式管理函数
void playSound(SoundPattern pattern)
{
    switch (pattern)
    {
    case ALERT_FOOD_LOW:
        // 储粮不足：放音调序列
        buzz.freq(262, BEAT_1);
        break;

    case ALERT_BOWL_FULL:
        // 食盆已满：播放音调序列
        buzz.freq(330, BEAT_1);
        break;

    case ALERT_PET_ABSENT:
        // 宠物离开：播放音调序列
        buzz.freq(392, BEAT_1);
        break;

    case ALERT_HARDWARE_ERROR:
        // 硬件错误：播放音调序列
        buzz.freq(523, BEAT_1);
        break;

    case SOUND_FEED_SUCCESS:
        // 喂食成功：播放音调序列
        buzz.freq(659, BEAT_1);
        break;

    case SOUND_PORTION_CHANGE:
        // 份量切换：播放音调序列
        buzz.freq(784, BEAT_1);
        break;
    }
}

// 修改显示函数以包含在线状态
void displayStatus()
{
    static String lastDisplayContent = ""; // 用于存储上次显示的内容
    char currentDisplayContent[128]; // 增加缓冲区大小以适应多行内容

    // 第一行：时间
    snprintf(currentDisplayContent, sizeof(currentDisplayContent), "时间: %02lu:%02lu\n",
             (millis() / 3600000) % 24,
             (millis() / 60000) % 60);

    // 第二行：份量
    char portionStr[32];
    snprintf(portionStr, sizeof(portionStr), "份量: %s\n", getCurrentPortionText());
    strcat(currentDisplayContent, portionStr);

    // 第三行：在线状态
    char onlineStr[32];
    snprintf(onlineStr, sizeof(onlineStr), "状态: %s\n", isOnline ? "在线" : "离线");
    strcat(currentDisplayContent, onlineStr);

    // 第四行：储粮信息
    char statusStr[32];
    snprintf(statusStr, sizeof(statusStr), "储粮剩余: %.1fg\n", remainingFood);
    strcat(currentDisplayContent, statusStr);

    // 第五行：网络状态（仅在离线时显示重连信息）
    if (!isOnline && reconnectAttempts > 0)
    {
        char reconnectStr[32];
        snprintf(reconnectStr, sizeof(reconnectStr), "重连: %d次", reconnectAttempts);
        display.setCursor(0, 64); // 调整Y坐标以适应新行
        display.printLine(reconnectStr);
    }

    // 仅在显示内容变化时更新显示
    if (lastDisplayContent != currentDisplayContent)
    {
        display.fillScreen(0); // 清屏
        display.setCursor(0, 0);
        display.print(currentDisplayContent);
        lastDisplayContent = currentDisplayContent; // 更新上次显示内容
    }
}

// IoT消息处理函数
void processIotMessages()
{
/*     if (!isOnline)
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
    } */
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

// 添加错误显示函数（之前被引用但定义）
void displayError(const char *error)
{
    display.fillScreen(0);
    display.setCursor(0, 0);
    display.print("错误:");
    display.setCursor(0, 16);
    display.print(error);
}

// 3. 主程序函数
void setup()
{
    Serial.begin(115200);
    Wire.begin();

    // 初始化显示
    display.begin();       // 初始化显示
    display.fillScreen(0); // 清屏

    feedServo.attach(SERVO_PIN);
    feedServo.angle(0);

    while (!bowlScale.begin())
    {
        playSound(ALERT_HARDWARE_ERROR);
        displayError("称重模块错误");
        delay(2000);
    }

    // 设置固定的校准因子
    bowlScale.setCalibration(1677.f); // 固定校准因子为1677
    bowlScale.peel();

    pinMode(BUTTON_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT); // 设置LED引脚为输出模式

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
    // checkPetApproach();
    // checkPetAbsence();

    delay(100);
}

