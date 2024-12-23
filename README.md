# 智能宠物喂食器

## 项目信息
- **参赛项目**：2024年上海市少年儿童乐创挑战系列活动
- **参赛队名**：智慧星河队
- **学校**：上海市静安区一中心小学
- **班级**：三（6）班
- **队员**：王一野、乔安易、程云骞、夏长榎
- **指导老师**：陈晓婧、丁毅
- **报名编号**：TFJ10037

## 项目背景
随着现代生活节奏的加快，越来越多的家庭选择饲养宠物作为家庭成员，但工作繁忙的主人往往难以保证按时给宠物喂食。传统的宠物喂食器功能单一，缺乏智能监控功能，无法及时发现和解决喂食过程中的问题。

我们的智能宠物喂食器针对以下问题提供解决方案：
1. **定时喂食**：解决主人无法按时喂食的问题
2. **远程控制**：随时查看和控制喂食情况
3. **智能监测**：实时监控储粮和宠物状态
4. **异常预警**：及时发现并提醒各类异常情况

本项目采用掌控板作为主控，结合多种传感器和云平台，实现了一个功能完整、操作简便的智能喂食解决方案。不仅能确保宠物按时进食，还能帮助主人随时了解宠物的喂食情况，让宠物照料更加智能化、人性化。

## 功能特点
1. 自动喂食
   - 定时喂食：每8小时自动投放一次
   - 手动喂食：通过按钮触发
   - 远程喂食：通过Easy IoT平台控制
   - 可调节份量：小份(20g)、中份(50g)、大份(100g)
   - 食盘防溢出保护

2. 智能监测
   - 储粮监测：计算剩余量并预警
   - 宠物监测：检测宠物进食情况
   - 异常报警：储粮不足、宠物长时间未进食等
   - 状态上报：定期向云端报告运行状态

3. 人机交互
   - OLED显示：实时显示运行时间和在线状态
   - 按键操作：
     - 单击按钮：执行喂食
     - 长按按钮：进入称重校准模式
     - 单击（ButtonA）：切换份量（小份/中份/大份）
     - 单击（ButtonB）：重置储粮量
   - 声音提示：不同状态对应不同声音模式
   - 远程控制：通过Easy IoT平台操作

## IoT功能说明
1. 在线功能
   - 远程喂食控制
   - 远程调整份量
   - 实时状态监控
   - 异常情况推送

2. 离线模式
   - 自动切换离线模式
   - 保持基本喂食功能
   - 显示离线状态
   - 智能重连机制

3. 网络管理机制
   - 智能重连策略
     * 初始5秒重试间隔
     * 最大5分钟重试间隔
     * 指数退避算法
     * 连接稳定后重置参数
   
   - 状态监控
     * 实时连接状态追踪
     * 自动重新订阅主题
     * 断线自动处理
     * 重连进度显示

   - 稳定性保护
     * 最大重试次数限制
     * 智能延迟机制
     * 资源消耗控制
     * 自动恢复机制

4. 状态上报内容
   - 剩余储粮量
   - 当前份量设置
   - 食盆重量
   - 设备在线状态
   - 网络连接质量

## 声音提示说明
1. 警告类提示
   - 储粮不足：三声长鸣（500ms x 3）
   - 食盆已满：两短一长（100ms + 100ms + 500ms）
   - 宠物离开：四声短鸣（100ms x 4）
   - 硬件错误：一长两短（500ms + 100ms + 100ms）

2. 操作类提示
   - 喂食成功：一声短鸣（200ms）
   - 份量切换：两声短鸣（100ms x 2）
   - 重置储粮：三声短鸣后一声长鸣

## 硬件要求
- 掌控板(支持Arduino IDE和Mind+编程环境)
- 舵机（投食机构）
- DFRobot_HX711_I2C称重模块
- 超声波测距模块
- 蜂鸣器
- OLED显示屏（掌控板自带）
- WiFi连接（掌控板自带）

## 引脚连接
- P0: 多功能按钮（按键输入）
- P16: 蜂鸣器（声音提示）
- P2: 超声波Trig（距离检测发送）
- P3: 超声波Echo（距离检测接收）
- P15: 舵机控制（投食机构）
- I2C接口: 称重模块（地址0x64）

注意：所有引脚定义都使用掌控板标准的"P+数字"命名方式，这是掌控板的硬件要求。

## 使用说明
1. 首次配置：
   - 修改WiFi信息（WIFI_SSID和WIFI_PASSWORD）
   - 配置Easy IoT设备ID和密码
   - 上传程序并等待自动连接
   - 进行称重校准

2. 按键操作：
   - 单击：执行喂食
   - 双击：切换份量（小份/中份/大份）
   - 三连击：重置储粮量
   - 长按：进入称重校准模式

3. 显示界面：
   - 第一行：运行时间、当前份量和在线状态
   - 第二行：状态信息和提示

4. IoT平台操作：
   - 远程触发喂食
   - 远程调整份量
   - 查看设备状态
   - 接收异常警报

5. 日常维护：
   - 定期检查储粮情况
   - 清理食盆
   - 检查机构是否正常

## 依赖库
- Wire.h：I2C通信
- Servo.h：舵机控制
- ezButton.h：按键处理
- DFRobot_HX711_I2C.h：称重模块
- MPython.h：掌控板核心库
- DFRobot_Iot.h：Easy IoT功能

## 代码结构
1. 基础配置
   - 包含库文件
   - 常量定义
   - 对象创建
   - IoT配置

2. 核心功能
   - 喂食相关函数
   - 显示相关函数
   - 按钮处理函数
   - 传感器相关函数
   - IoT通信函数

3. 主程序
   - setup(): 初始化配置
   - loop(): 主循环逻辑

## 注意事项
1. 使用限制
   - 储粮量计算基于累计投放量
   - 重启后所有数据重置
   - 每8小时自动喂食一次
   - 需要稳定的WiFi环境

2. 网络连接管理
   - 初始连接：首次启动时自动连接
   - 断线重连：智能间隔尝试重连
   - 稳定性：5分钟稳定连接后重置参数
   - 资源控制：避免频繁重连消耗

3. 可能的问题
   - 储粮量计算可能有误差
   - 宠物检测可能受环境影响
   - 按键可能需要适应才能熟练操作
   - 网络不稳定时会自动进入离线模式
   - 重连过程可能暂时影响响应速度

4. 改进建议
   - 添加数据存储功能保存设置
   - 增加更多传感器监测
   - 进一步优化网络重连策略
   - 添加网络质量监测
   - 实现本地数据缓存

5. 网络安全
   - 定期更换IoT密码
   - 避免在公共网络使用
   - 注意保护设备ID和密码
   - 定期检查异常访问记录
   - 监控网络连接状态

## 故障排除

1. 网络连接问题
   - 检查WiFi信息是否正确
   - 观察重连次数和间隔
   - 确认路由器工作正常
   - 检查IoT平台服务状态

2. 显示问题
   - 确认网络状态显示正确
   - 检查重连进度显示
   - 验证错误信息显示
   - 观察状态更新及时性

3. 功能异常
   - 在线模式功能异常时尝试重启
   - 离线模式下确认基本功能
   - 检查传感器工作状态
   - 验证网络切换是否正常

## 维护建议

1. 日常维护
   - 定期检查网络连接状态
   - 观察重连日志和频率
   - 确认在线/离线模式切换
   - 验证状态上报正常

2. 定期检查
   - 网络连接质量
   - 重连策略效果
   - 在线时间统计
   - 异常记录分析

3. 优化调整
   - 根据使用环境调整重连参数
   - 优化网络连接策略
   - 更新固件版本
   - 改进异常处理机制
