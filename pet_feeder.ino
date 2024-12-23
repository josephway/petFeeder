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
 *    - 可调节份量：支持小份(20g)、中份(50g)、大份(100g)
 *    - 食盘防溢出：实时监测食盘重量，超过阈值自动停止投食
 *
 * 2. 智能监测
 *    - 储粮监测：通过累计投放量计算剩余量，及时提醒补充
 *    - 宠物监测：通过超声波传感器检测宠物是否正常进食
 *    - 在线状态：支持WiFi连接，可远程监控设备状态
 *
 * 3. 按键操作
 *    - 单击：执行喂食
 *    - 双击：切换份量（小份/中份/大份）
 *    - 三连击：重置储粮量
 *    - 长按：进入称重校准模式
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
#define IOT_ID "你的设备ID"
#define IOT_PWD "你的设备密码"
#define WIFI_SSID "你的WiFi名称"
#define WIFI_PASSWORD "你的WiFi密码"

// 定义IoT主题
#define TOPIC_FEED "feeder/cmd/feed"       // 远程喂食指令
#define TOPIC_PORTION "feeder/cmd/portion" // 远程调整份量
#define TOPIC_STATUS "feeder/status"       // 状态上报

// 引脚定义 - 掌控板专用引脚命名
#define BUTTON_PIN P0  // 多功能按钮输入引脚（支持单击/双击/三击/长按）
#define BUZZER_PIN P16 // 蜂鸣器输出引脚（用于声音提示）
#define TRIG_PIN P2    // 超声波模块Trig引脚（发送信号）
#define ECHO_PIN P3    // 超声波模块Echo引脚（接收信号）
#define SERVO_PIN P15  // 舵机控制引脚（控制投食机构）

// I2C设备配置
#define HX711_I2C_ADDR 0x64 // DFRobot HX711称重模块I2C地址（使用默认地址）

// 重量相关阈值
#define MAX_FOOD_WEIGHT 500.0             // 食盆最大重量（克）
#define MIN_STORAGE_WEIGHT 100.0          // 储粮桶最小重量（克）
#define LOW_THRESHOLD (PORTION_LARGE * 2) // 缺粮警告阈值

// 喂食份量定义
#define PORTION_SMALL 20  // 小份量(g)
#define PORTION_MEDIUM 50 // 中份量(g)
#define PORTION_LARGE 100 // 大份量(g)
#define SERVO_ANGLE 90    // 舵机转动角度

// 距离检测相关
#define MAX_DISTANCE 200      // 超声波最大测量距离(cm)
#define DISTANCE_THRESHOLD 20 // 宠物检测阈值距离(cm)
#define PET_ABSENCE_HOURS 12  // 宠物离开警告时间阈值（小时）

// 新增按钮相关常量
#define DEBOUNCE_TIME 50           // 按钮消抖时间(ms)
#define CLICK_TIMEOUT 500          // 点击超时时间(ms)
#define LONG_PRESS_TIME 2000       // 长按时间(ms)
#define INITIAL_FOOD_AMOUNT 1000.0 // 初始储粮量(g)

// 喂食间隔设置
#define FEEDING_INTERVAL 28800000 // 喂食间隔(8小时 = 8 * 60 * 60 * 1000 ms)
#define FEEDING_TIMES_PER_DAY 3   // 每天喂食次数

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
unsigned long lastReleaseTime = 0;
int clickCounter = 0;
float remainingFood = INITIAL_FOOD_AMOUNT; // 剩余储粮量

// 喂食计时
unsigned long lastFeedingTime = 0; // 上次喂食时间

// 创建IoT对象
DFRobot_Iot myIot;

// 网络状态管理
#define MAX_RECONNECT_ATTEMPTS 5      // 最大重连次数
#define INITIAL_RETRY_DELAY 5000      // 初始重试延迟(5秒)
#define MAX_RETRY_DELAY 300000        // 最大重试延迟(5分钟)
#define STABLE_CONNECTION_TIME 300000 // 稳定连接时间(5分钟)

// 网络状态变量
bool isOnline = false;
unsigned long lastConnectionAttempt = 0;
unsigned long connectionEstablishedTime = 0;
int reconnectAttempts = 0;
unsigned long currentRetryDelay = INITIAL_RETRY_DELAY;

// 声明变量
unsigned long lastPetVisitTime = 0;

// 假设 buttonA 和 buttonB 是某种按钮类的实例
Button buttonA;
Button buttonB;

// 假设 display 是种显示类的实例
Display display;

// 移动显示相关函数到一起
void displayMessage(const char *line1, const char *line2 = "")
{
    display.fillScreen(0); // 清屏
    display.setCursor(0, 0);
    display.print(line1); // 第一行文字
    if (strlen(line2) > 0)
    {
        display.setCursor(0, 16);
        display.print(line2); // ���二行文字
    }
    display.show(); // 显示
}

// 网络连接管理类
class NetworkManager
{
private:
    unsigned long lastStatusCheck = 0;
    bool wasOnline = false;

public:
    // 检查并管理网络连接
    void manage()
    {
        unsigned long currentTime = millis();

        // 定期检查连接状态
        if (currentTime - lastStatusCheck >= 10000)
        { // 每10秒检查一次
            lastStatusCheck = currentTime;

            if (isOnline != wasOnline)
            {
                wasOnline = isOnline;
                if (isOnline)
                {
                    onConnectionEstablished();
                }
                else
                {
                    onConnectionLost();
                }
            }

            // 检查连接是否稳定
            if (isOnline && connectionEstablishedTime > 0)
            {
                if (currentTime - connectionEstablishedTime >= STABLE_CONNECTION_TIME)
                {
                    // 连接稳定，重置重连参数
                    resetReconnectionParams();
                }
            }
        }

        // 处理重连
        if (!isOnline && shouldAttemptReconnection(currentTime))
        {
            attemptReconnection();
        }
    }

private:
    // 重置重连参数
    void resetReconnectionParams()
    {
        reconnectAttempts = 0;
        currentRetryDelay = INITIAL_RETRY_DELAY;
    }

    // 判断是否应该尝试重连
    bool shouldAttemptReconnection(unsigned long currentTime)
    {
        if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS)
        {
            // 达到最大重试次数，增加延迟
            if (currentTime - lastConnectionAttempt >= currentRetryDelay)
            {
                currentRetryDelay = min(currentRetryDelay * 2, MAX_RETRY_DELAY);
                reconnectAttempts = 0;
                return true;
            }
        }
        else if (currentTime - lastConnectionAttempt >= currentRetryDelay)
        {
            return true;
        }
        return false;
    }

    // 尝试重新连接
    void attemptReconnection()
    {
        lastConnectionAttempt = millis();
        reconnectAttempts++;

        displayMessage("重新连接中...",
                       String("第" + String(reconnectAttempts) + "次尝试").c_str());

        if (connectToIot())
        {
            isOnline = true;
            connectionEstablishedTime = millis();
        }
        else
        {
            isOnline = false;
        }
    }

    // 连接建立时的处理
    void onConnectionEstablished()
    {
        displayMessage("网络已连接", "同步中...");
        connectionEstablishedTime = millis();

        // 立即上报状态
        reportStatus();

        // 重新订阅主题
        myIot.subscribe(TOPIC_FEED);
        myIot.subscribe(TOPIC_PORTION);
    }

    // 连接断开时的处理
    void onConnectionLost()
    {
        displayMessage("网络已断开", "切换离线模式");
        connectionEstablishedTime = 0;

        // 可以在这里添加其他断线处理逻辑
    }
};

// 创建网络管理器实例
NetworkManager networkManager;

// 修改连接函数
bool connectToIot()
{
    if (myIot.wifiConnect(WIFI_SSID, WIFI_PASSWORD))
    {
        displayMessage("WiFi已连接", "连接IoT中...");
        delay(1000);

        if (myIot.init(IOT_ID, IOT_PWD))
        {
            return true;
        }
    }
    return false;
}

// 2. 核心功能函数
// 移动喂食相关函数到一起
void feed()
{
    float bowlWeight = bowlScale.readWeight();
    float portionWeight = getCurrentPortionWeight();

    if (bowlWeight >= MAX_FOOD_WEIGHT)
    {
        playSound(ALERT_BOWL_FULL);
        displayMessage("食盆已满", "暂停投食");
        return;
    }

    if (remainingFood <= MIN_STORAGE_WEIGHT)
    {
        playSound(ALERT_FOOD_LOW);
        displayMessage("���粮不", "请及时补充");
        return;
    }

    // 执行喂食
    feedServo.write(SERVO_ANGLE);
    delay(2000);
    feedServo.write(0);

    remainingFood -= portionWeight;

    playSound(SOUND_FEED_SUCCESS);
    displayMessage("喂食完成",
                   String("剩余约: " + String(remainingFood, 1) + "g").c_str());
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

    // 最后一声长响表示完成
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
}


// 移动按钮处理相关函数到一起
void handleButton()
{
    static bool isLongPress = false;
    static unsigned long pressStartTime = 0;

    // 使用掌控板内建的ButtonA和ButtonB
    if (digitalRead(BUTTON_PIN) == LOW) // 检测P0引脚的单击
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

    // 使用ButtonA的单击替代双连击功能
    if (buttonA.isPressed())
    {
        adjustPortion();
    }

    // 使用ButtonB的单击替代三连击功能
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
        // 储粮不足：三长音
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

    // 第一行：时间和在线状态
    char timeStr[32];
    sprintf(timeStr, "%02d:%02d %s [%s]",
            (millis() / 3600000) % 24,
            (millis() / 60000) % 60,
            getCurrentPortionText(),
            isOnline ? "在线" : "离线");
    display.setCursor(0, 0);
    display.print(timeStr);

    // 第二行：储粮信息
    char statusStr[32];
    sprintf(statusStr, "储粮:%dg", (int)remainingFood);
    display.setCursor(0, 16);
    display.print(statusStr);

    // 第三行：宠物状态
    char petStr[32];
    sprintf(petStr, "上次:%d分钟前", (millis() - lastPetVisitTime) / 60000);
    display.setCursor(0, 32);
    display.print(petStr);

    // 第四行：网络状态
    if (!isOnline && reconnectAttempts > 0)
    {
        char reconnectStr[32];
        sprintf(reconnectStr, "重连:%d次", reconnectAttempts);
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

// 3. 主程序函数
void setup()
{
    Serial.begin(115200);
    Wire.begin();

    // 始化显示
    display.begin()     // 初始化显示
        display.fill(0) // 清屏
        display.show()

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
}

void loop()
{
    // 1. 更新系统状态
    handleButton(); // 调用新的按钮处理函数
    unsigned long runTime = millis();

    // 网络管理
    networkManager.manage();

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
