Pacemaker
0.36
第 1 轮 0.0，其余 0.4


1. 生成范式消融实验（RQ3）：针对案例Time_trigger、MinePump、Radar、Flight_Controller和ROSACE，将本文的混合生成框架替换为纯LLM端到端生成基线。通过对比，分析模板化机制与契约约束在抑制幻觉、保障结构准确性方面的有效性。在该实验中，纯LLM配置的实验的输入仍然是经过架构转换的ROS 2架构契约。为了保证共享变量和额外代码可以在组件代码中正确实现，该实验中的组件和节点的提示词中嵌入了对共享变量和额外代码的正确引用提示。 这一段话应该如何组织？

LLM生成的话出现的错误多的是：

1. Qos配置出错，引用ROS 2不存在的Qos配置方式；
2. 共享变量的引用出错，引用不存在的共享变量，大小写也不一致。
3. 尽管有hpp作为约束，引用的hpp路径也会出错；
4. API调用错误，发送者和订阅者的API调用没有符合ros 2 jazzy的标准来。
5. 组件名称错误，忽略了大小写。

低表现区间 (F1 < 0.80)
Ardupilot_Map (Run 7)
DeviceControllerMonitor (Run 1)
radar (Run 7)
robot (Run 8)

中等表现区间 (0.80 ≤ F1 < 0.95)
Flight_Controller (Run 8)
redundant_system — [抽样 Run 6]
AirConditioner (Run 3)
ROSACE_XtratuM — [抽样 Run 8]
tt (Run 1)

高表现区间 (F1 ≥ 0.95)
door_management (Run 2)
MinePump (Run 5)
PC_Simple (Run 9)

265条

专家1：
专家2：
分歧：10条

要求ai率为5%以下，逻辑要严谨，语句要通顺，要求口语化占比30%，学术用语占70%。


| Complexity    | System name                        | Devices | Processes | Threads | BA  | Subprograms | Shared variable | Description                                      |
| ------------- | ---------------------------------- | ------- | --------- | ------- | --- | ----------- | --------------- | ------------------------------------------------ |
| Simple        | Ardupilot_code                     | 0       | 1         | 4       | ×   | √ (C)       | ×               | 单进程纯软件逻辑，无设备交互                                   |
|               | time_triggered                     | 0       | 1         | 3       | ×   | √ (Ada)     | √               | 时间触发流水线                                          |
|               | pacemaker（DeviceControllerMonitor） | 1       | 1         | 3       | √   | ×           | ×               | 单进程纯BA                                           |
|               | Producer_Consumer（PC_Simple）       | 0       | 2         | 2       | ×   | √(Ada)      | √               |                                                  |
| Complex       | redundant_system                   | 0       | 1         | 5       | √   | ×           | ×               | 单进程纯BA                                           |
|               | AirConditioner                     | 3       | 1         | 3       | √   | ×           | ×               | 单进程纯BA                                           |
|               | fcc (飞行控制)                         | 1       | 1         | 14      | √   | √（C）        | ×               | 线程密集型系统。包含少量状态变化，主打高并发节点调度与逻辑解算。                 |
|               | minepump_ba (矿井水泵)                 | 0       | 1         | 4       | √   | √ (C)       | √               | 逻辑混合型系统。兼具模型层面的状态转移(BA)与底层C逻辑，同时要求处理并发共享内存。      |
|               | radar (雷达片段)                       | 3       | 1         | 5       | ×   | √(Ada)      | √               | 硬件数据驱动型系统。多设备输入，依赖异构语言(Ada)代码集成与共享状态同步。          |
|               | robot_ba (双轮机器人)                   | 0       | 4         | 6       | √   | ×           | ×               | 分布式IPC系统。多进程间需通过ROS2底层机制进行跨边界通信，纯依赖BA驱动行为。       |
| Super Complex | rosace                             | 0       | 3         | 11      | ×   | √           | √               | 很复杂                                              |
|               | doors (A380电动舱门)                   | 7       | 1         | 3       | √   | ×           | ×               | 状态爆炸型系统。与大量外部设备(传感器/执行器)存在高度交互，包含海量且极度复杂的并发状态转移逻 |


1. 我们解决了行为语义在AADL到底层代码的跨层转换中的丢失问题
2. 我们定义了AADL行为属性到ROS2状态机的映射规则，并用LLM处理规则无法覆盖的歧义部分
3. 我们提出了面向生成代码的运行时语义验证框架，能检测编译期不可见的并发错误

这个工作最有价值的地方不是"AADL转ROS2"这件事本身，而是**"在行为语义映射不完整的条件下，如何用LLM+运行时验证构建可靠的代码生成闭环"**——这个问题比AADL和ROS2都更通用，也更有学术生命力。

实验问题：
RQ1：系统能否正确完成AADL到ROS2的转换（包括BA和子程序）
每个进行10次代码生成，判断编译成功率
RQ2：多环验证能否有效检测并修复错误？（分析时序错误和共享内存以及复杂的状态变化doors，以及多进程之间的传输）B组，复杂组
RQ3：与现有方法相比有何优势？分为只用模板和只用LLM生成组件代码两组。fcc和minepump_ba
RQ4: 大模型无关实验（可有可无）

实验对象：
A组，简单组   每个10次，对照组
RQ1：系统能否正确完成AADL到ROS2的转换（包括BA和子程序）？简单模型
time_triggered:ada语言编写子程序代码，不包含BA，单进程，3个线程，包含所有属性 简单  ok
ardupilot_software：多设备，单进程，多线程，具有子程序，无BA，无优先级。 简单。  ok
radar：单进程，多线程，有子程序，但是没有BA。包含所有属性。 简单 ok

B组，复杂组
RQ1：系统能否正确完成AADL到ROS2的转换（包括BA和子程序）？复杂模型
Robot_BA: 多进程，多线程，没有子程序，只有BA， 只包含周期和执行时间  复杂 ok
minepump_ba：单进程，多线程，有BA，有子程序，没有截止时间。复杂！好用！  ok
    存在线程中存在共享变量问题：pthread_mutex_t mutex_simu_minepump;
int CmdPump_Value = 0; /* c = 0, pump is off, c = 1, pump is on */
int WaterLevel_Value = 50;
加入了共享变量的处理。
doors：单进程，多线程，具有复杂的BA，无子程序，无截止时间和优先级。复杂BA！   wait
    device中也存在BA。暂时还没有处理。状态变化太过复杂，需要专用的测试用例，随机用例解决不了
fcc：单进程，多线程，有子程序和BA，包含所有属性。复杂  ok！
时序检测经常报错，但是可以解决

消融实验：RQ3：多环验证能否有效检测并修复生成代码中的错误？
验证：
  fcc的时序报错 → 超时检测的必要性
  minepump_ba的共享变量死锁 → 死锁检测的必要性
  两个案例直接支撑这个RQ

./example/ardupilot, main_Ardupilot.aadl, Ardupilot_Map, ./test_set/Ardupilot_Map

./example/doors, dms.aadl, door_management, ./test_set/door_management

./example/fcc, Flight_Controller.aadl, Flight_Controller, ./test_set/Flight_Controller

./example/minepump_ba, minepump_ba.aadl, MinePump, ./test_set/MinePump
MethaneLevel: out data port Int; 改为了MethaneLevel: out event data port Int;
WaterAlarm: out data port Int; 改为了WaterAlarm: out event data port Int; *2

./example/pacemaker, pacemaker.aadl, DeviceControllerMonitor, ./test_set/DeviceControllerMonitor
包名Pacemaker_Pkg改为了pacemaker
把vrp: data int {Data_Model::Initial_Value => ("1");}; 改为vrp: data int {Data_Model::Initial_Value => ("0");};

./example/producer_consumer, producer_consumer.aadl, PC_Simple, ./test_set/PC_Simple
删除了real_specification

./example/radar, radar.aadl, radar, ./test_set/radar
将原代码的package名：radar_system修改为了radar

./example/redundancy, redundant_system.aadl, redundant_system, ./test_set/redundant_system
将package名称从redundant_system_Pkg改为redundant_system
添加：
data int
properties
  Data_Model::Data_Representation => Integer;
end int;

./example/regulator, air_conditioner.aadl, AirConditioner, ./test_set/AirConditioner
将package名称从AirConditioner_Pkg改为Air_Conditioner
添加了int
WITH Data_model;
data int
properties
  Data_Model::Data_Representation => Integer;
end int;

将HeatRegulator删除，将HeaterSW、HeaterCPU和HeaterRAM移到AirConditioner中。
将cnx_1修改为HeaterSW.measuredTemp -> Temperature.value;
添加了三个设备
device IntSelector
features
  value : OUT DATA PORT int;
end IntSelector;

device IntDisplay
features
  value : IN DATA PORT int;
end IntDisplay;

device Light
features
  red : IN EVENT PORT;
  green : IN EVENT PORT;
end Light;

./example/robot_ba, robot_ba.aadl, robot, ./test_set/robot
将Alpha_Type全部修改为了Base_Types::Boolean

./example/rosace, rosace-xtratum.aadl, ROSACE_XtratuM, ./test_set/ROSACE_XtratuM
将连接C9 : port Va_c -> Va.Va_c; 删除了

./example/time_triggered, time_triggered.aadl, tt, ./test_set/tt