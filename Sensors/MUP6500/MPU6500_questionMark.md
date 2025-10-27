# MPU6500问题记录

## 关于FSYNC的总结
    
### FSYNC 功能核心总结

FSYNC （synchronize）引脚的核心功能是同步。但它有两种工作模式，你一次只能选择一种来使用：

**数据同步模式 (Data Synchronization)**： 外部设备通过 FSYNC 引脚告诉 MPU6500 “某个时刻发生了某事”，MPU6500 会把这个瞬间标记在它采集到的传感器数据上。

**中断输入模式 (Interrupt Input)**： 外部设备通过 FSYNC 引脚向 MPU6500 “发送一个中断信号”，触发 MPU6500 内部的某个动作或通知主机。

这两种模式是**互斥**的，你需要通过配置对应的寄存器来选择用哪一种。

### FSYNC 有关寄存器以及两种模式介绍

由于FSYNC主要为两种互斥的模式，所以这里分不同模式所需不同寄存器来介绍。

#### 同步模式

![FSYNC1](//Photos/MPU6500_questionMark/FSYNC1.png)

MPU6500 的同步模式的配置仅由配置寄存器（Register 26 – Configuration）控制。

MPU6500 的配置寄存器（Register 26 – Configuration）中的 EXT_SYNC_SET[2:0] 位（第5位到第3位）用于配置 FSYNC（Frame Synchronization）引脚 的同步功能。FSYNC 是一个同步信号输入引脚，可用于外部设备（如相机、其他传感器或主控制器）与 MPU6500 进行时间同步。

EXT_SYNC_SET[2:0] 的配置决定了 FSYNC 信号被采样到哪个传感器的数据低位（LSB），从而实现外部事件与传感器数据的同步。

FSYNC 引脚输入一个同步脉冲信号（通常是一个短脉冲）。

MPU6500 会在每个采样周期捕获该信号，并将其锁存到指定传感器的数据最低位（LSB）。

这样，你可以在读取传感器数据时，通过检查该位的值（0或1）来判断是否在采样时刻发生了外部同步事件。

#### 中断模式

MPU6500 的中断模式的配置主要由中断配置寄存器（Register 55 – INT_PIN_CFG）和中断使能寄存器（Register 56 – INT_ENABLE）控制。

1. 中断配置寄存器（Register 55 – INT_PIN_CFG）

    ![FSYNC2](photos\FSYNC2.png)

    这个寄存器有两个位专门控制 FSYNC 的中断模式：

    位 [2] - FSYNC_INT_MODE_EN (FSYNC中断模式使能)

    功能：FSYNC 功能的“总开关”。

    怎么用：

    = 1：开启【中断输入模式】。此时 FSYNC 引脚变成一个中断输入引脚，EXT_SYNC_SET 的设置无效。(**自动使FSYNC的同步模式无效**)

    = 0：关闭中断输入模式。此时你可以使用 CONFIG 寄存器的【数据同步模式】。

    位 [3] - ACTL_FSYNC (FSYNC有效电平)

    功能：设置 FSYNC 中断信号的“脾气”。

    怎么用（需要先设置 FSYNC_INT_MODE_EN = 1）：

    = 1：FSYNC 引脚为低电平有效（即低电平时表示有中断）。

    = 0：FSYNC 引脚为高电平有效（即高电平时表示有中断）。

2. 中断使能寄存器（Register 56 – INT_ENABLE）

    ![FSYNC3_1](photos\FSYNC3_1.png)
    ![FSYNC3_2](photos\FSYNC3_2.png)

    INT_ENABLE 位 [3] - FSYNC_INT_EN

    功能：是否允许 FSYNC 中断信号传递到 MPU6500 的 INT (中断) 引脚上。

    怎么用：

    = 1：允许。当 FSYNC 引脚有中断信号时，MPU6500 的 INT 引脚也会输出信号，从而通知主处理器（如单片机）。

    = 0：不允许。即使 FSYNC 有中断，INT 引脚也没反应。

### 如何使用：两种模式的配置步骤

#### 场景一：我想用 FSYNC 标记数据（例如同步相机快门）
配置模式：确保 INT_PIN_CFG 的 FSYNC_INT_MODE_EN = 0（关闭中断模式）。

选择同步目标：在 CONFIG 寄存器中，设置 EXT_SYNC_SET 的值。例如，设为 4（同步到陀螺仪Z轴）。

读取数据：正常读取传感器数据（如 GYRO_ZOUT_L 和 GYRO_ZOUT_H）。

检查标记：检查 GYRO_ZOUT_L 的最低位（bit 0），它的值就反映了FSYNC信号在采样时刻的状态。

*实际场景*

假如说现在我有两块MPU6500，一块离单片机近，一块离单片机远，假如我现在想要将两块传感器的数据进行数据融合，由于两个传感器离单片机距离不一样，当读取一组数据的时候肯定会有一定差距，此时就必须要用FSYNC来进行数据同步。

    工作流程：

    发起同步采样：当单片机需要一次同步采样时，它控制那个GPIO口，产生一个短暂的脉冲（例如，从低电平跳变到高电平再跳变回低电平）。

    传感器同时动作：两块MPU6500的硬件同时在FSYNC引脚上检测到这个脉冲的边沿（例如上升沿）。

    同时采样：两块MPU6500的数据路径会在下一个采样周期到来时，将这个同步信号的状态“烙”在指定的传感器数据位上。关键是，这个“下一个采样周期”对两个传感器来说是同一个时刻，因为它们使用相同的内部时钟和配置。

    读取数据：单片机可以像平常一样，依次读取两块传感器的数据（先读近的，再读远的，没关系）。

    数据配对与验证：当单片机读取到数据后，它可以检查传感器数据的最低位（LSB，例如GYRO_ZOUT_L[0]）。

    如果这一组数据的最低位都是 1（或都是 0），说明它们是在同一个同步脉冲期间采样的，是真正意义上“同一时刻”的数据，可以用于融合。

    你可以设计你的同步脉冲：例如，第一次脉冲为高电平，下一次脉冲为低电平，这样就能区分不同批次的数据。

#### 场景二：我想把 FSYNC 当作一个普通的中断输入引脚
配置模式：设置 INT_PIN_CFG 的 FSYNC_INT_MODE_EN = 1（开启中断模式）。

设置触发方式：设置 INT_PIN_CFG 的 ACTL_FSYNC，决定高电平还是低电平触发。

使能中断输出（可选）：如果你希望这个中断能通知主处理器，设置 INT_ENABLE 的 FSYNC_INT_EN = 1。

检测中断：当 FSYNC 引脚有信号时，读取 INT_STATUS 寄存器，检查 FSYNC_INT 位是否为 1。

*实际场景*

FSYNC作中断模式时就是一个简单的经过MPU6500 的一个中断，且这个中断不会对MPU6500 的任何运行事件有影响，相对来说比较鸡肋。但是你可以将你的信号连接到FSYNC引脚，通过内部链接MPU6500 中的FSYNC和INT或者直接读取I2C的FSYNC中断触发状态来获取这个外部中断信号。通常是用作PCB逻辑路线优化的。

如果你的系统对延迟极其敏感（纳秒级），或者追求极简和绝对可靠，那么直接接单片机IO是更好的选择。

如果你的系统IO紧张，或者你希望将安全信号与传感器数据在逻辑上关联起来，实现更模块化的设计，那么使用FSYNC中断是一个巧妙且高效的解决方案。

## 关于中断的介绍

### 中断有关寄存器介绍
   
你可以把它想象成一个 “呼叫铃”。这个呼叫铃可以因为很多种原因被按响，比如“菜好了”（数据就绪）、“锅满了”（FIFO溢出）、“来客人了”（运动唤醒）等等。你可以自由选择关心哪些事件，以及这个呼叫铃的响铃方式（是大声喊叫还是闪灯）。

1. 中断引脚配置寄存器 INT_PIN_CFG (Register 55)
    
    ![INT1](photos\INT1.png)
    
    这个寄存器不决定“什么事”会触发中断，而是决定中断引脚 “如何工 作”。

    位 [7] - ACTL (电平有效方式)

    功能：设置 INT 引脚的有效电平。

    = 1：低电平有效。当有中断时，INT 引脚输出低电平。

    = 0：高电平有效。当有中断时，INT 引脚输出高电平。(默认)

    位 [6] - OPEN (输出模式)

    功能：设置 INT 引脚的输出电路结构。

    = 1：开漏输出 (Open Drain)。需要外部上拉电阻才能输出高电平。好 处是可以方便地实现线与，多个开漏输出的设备可以将INT线连在一起， 任何一个设备都能将其拉低。

    = 0：推挽输出 (Push-Pull)。芯片自己可以强有力地输出高电平和低电 平。(默认)

    位 [5] - LATCH_INT_EN (中断锁存)

    功能：这是极其重要的一个位！

    = 1：锁存模式。一旦有中断发生，INT 引脚的电平会一直保持有效状 态，直到你读取 INT_STATUS 寄存器后才恢复无效状态。这确保了主机 绝不会错过任何一次中断。

    = 0：脉冲模式。中断发生时，INT 引脚只会产生一个 50us 的短脉冲， 然后就恢复无效状态。如果主机正在处理其他任务，可能会错过这个脉 冲。

    位 [4] - INT_ANYRD_2CLEAR (清除方式)

    功能：如何清除中断状态（从而让INT引脚恢复）。

    = 1：读取任何寄存器都会清除 INT_STATUS 寄存器的所有状态位。不推 荐！ 容易误操作。

    = 0：只有读取 INT_STATUS 寄存器本身才会清除状态位。这是安全且推 荐的设置。

2. 中断使能寄存器 INT_ENABLE (Register 56) 

    ![INT2_1](photos\INT2_1.png)
    ![INT2_2](photos\INT2_2.png)
    
    这个寄存器是中断源的总开关。每一位控制一种类型的中断是否被允许去 触发 INT 引脚。

    位 [6] - WOM_EN： 唤醒中断 (Wake-on-Motion)。当加速度计检测到 的运动超过 WOM_THR 寄存器设置的阈值时，产生中断。

    位 [4] - FIFO_OFLOW_EN： FIFO 溢出中断。当 FIFO 缓冲区已满， 但又有新数据要写入时，产生中断。

    位 [3] - FSYNC_INT_EN： FSYNC 中断。当 FSYNC 引脚（配置为中断 输入模式）收到信号时，产生中断。

    位 [0] - RAW_RDY_EN： 原始数据就绪中断。当所有被启用的传感器 （加速度计、陀螺仪、温度）完成一次新的采样，数据已经准备好被读取 时，产生中断。这是最常用的中断。

3. 中断状态寄存器 INT_STATUS (Register 58)

    ![INT3](photos\INT3.png)
    
    当 INT 引脚有效时，主机必须读取这个寄存器来查明具体是哪个事件引 起的中断。它的位定义与 INT_ENABLE 寄存器完全一一对应。

    怎么用：当你的单片机检测到 INT 引脚有效后，进入中断服务程序，第 一件事就是读取这个寄存器。

    如果 RAW_DATA_RDY_INT (位 [0]) 为 1，说明是新数据准备好了。

    如果 FIFO_OFLOW_INT (位 [4]) 为 1，说明是 FIFO 溢出了。

    ...以此类推。

    重要：读取这个寄存器的操作本身，会根据 INT_ANYRD_2CLEAR 的设置 来清除所有状态位，为下一次中断做准备。

### 如何使用：典型配置流程

假设你想要在数据准备好时，让 INT 引脚产生一个高电平脉冲来通知单片机。

配置引脚行为 (INT_PIN_CFG, Reg 55):

ACTL = 0 (高电平有效)

OPEN = 0 (推挽输出)

LATCH_INT_EN = 0 (脉冲模式，50us)

INT_ANYRD_2CLEAR = 0 (安全清除方式)

使能中断源 (INT_ENABLE, Reg 56):

RAW_RDY_EN = 1 (使能数据就绪中断)

其他位保持为 0（禁用其他中断）

单片机侧：

将 MPU6500 的 INT 引脚连接到单片机的一个外部中断引脚。

将该引脚配置为上升沿触发（因为我们是高电平有效的脉冲）。

在单片机的中断服务函数中，第一时间读取 INT_STATUS 寄存器以确认中断源并清除状态，然后开始读取传感器数据 (ACCEL_*OUT*, GYRO_*OUT*, TEMP_*OUT*)。

## 关于ODR的配置

此部分比较复杂，涉及到多个滤波器通道的选择，配置，甚至需要一点点小小的计算。

### MPU6500 ODR配置核心概念
    
从最顶层看，MPU6500 的ODR其实是分为两部分，一部分是GYRO的，一部分是ACCEL的。这两个的ODR完全是可以根据选择配置不一样的，不一定是相同的输出速率！！！

根据公式

**ODR = SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)**

数据的输出速率 = 采样率 = 内部采样率 / (1 + 采样率分频系数)

我们要求ODR现在有两个未知数，**内部采样率(Fs或Rate)**，**采样分频系数(Simple_Div)**。

    Fs是来自于GYRO中表标注的，Rate是来自ACCEL中表标注的，名字不一样但是实际用处是一样的，后文都的内部采样率都称Fs。

Fs可以通过GYRO和ACCEL获取分别的数值，所以在我们提出已知的ODR的情况下Simple_Div也能很简单地得出了。

所以反过来看，MPU6500 的ODR配置基本思路就是

- 计算并配置Simple_Div
- 选择合适的GYRO和ACCEL数据输出组

这样就可以配置出我们需要的ODR了。

### MPU6500 的滤波器结构

MPU6500 的滤波器可以简单看作成两层，一层是数字低通滤波器(DLPF或A_DLPF_CFG)，另一层是滤波器旁路控制(FCHOICE_B或ACCEL_FCHOICE_B)。这两层只能任选其一，不能同时使用。

![ODR1](photos\ODR1.png)
![ODR2](photos\ODR2.png)

上图分别为GYRO的配置图和ACCEL的配置图，分别展示了在不同的寄存器配置下的传感器输出带宽(Bandwidth)，延迟(Delay)，内部采样率(Fs或Rate)等。

GYRO所对应的是DLPF和FCHOICE_B，ACCEL所对应的是A_DLPF_CFG和ACCEL_FCHOICE_B。

一般来说旁路滤波器的速率都比较快，但是相应的噪声可能更加明显。一般是需要在单片机再额外处理再作运算使用的。而数字低通滤波器可以理解为MPU6500 事先处理到一定程度后再发出来的数据，虽然延迟较大，但是数据更加可靠，适合于低速率的设备。

由表可知滤波器旁路控制的优先级是要大于数字低通滤波器的，也就是说当滤波器旁路控制的值有效时，数字低通滤波器自动无效。

### 有关GYRO和ACCLE的配置的寄存器

我们还是按照前面所说的配置ODR的顺序来介绍寄存器。

1. 采样率分频器 (SMPLRT_DIV) 

    ![ODR3](photos\ODR3.png)

    此寄存器主要用于配置Simple_Div。

    Simple_Div为uint8_t(八位宽)，也就是0-255大小。

    将计算好的数值直接填进去就可以了。

2. 配置寄存器（Register 26 – Configuration）

    ![ODR4](photos\ODR4.png)

    主要看[2:0]的DLPF_CFG，此位为GYRO的数字低通滤波器配置。

3. 陀螺仪配置 （Register 27 – Gyroscope Configuration）

    ![ODR5](photos\ODR5.png)

    [7:5]分别为xyz轴陀螺仪的自检

    [4:3]为GYRO的量程选择

    [1:0]为FCHOICE_B，此位为GYRO的滤波器旁路控制配置。

4. 加速度计配置 （Register 28 – Accelerometer Configuration）

    ![ODR6_1](photos\ODR6_1.png)
    ![ODR6_2](photos\ODR6_2.png)

    [7:5]分别为xyz轴加速度计的自检

    [4:3]为ACCEL的量程选择

5. 加速度计配置2 （Register 29 – Accelerometer Configuration 2 ）

    ![ODR7](photos\ODR7.png)

    [3]为ACCEL_FCHOICE_B，此位为ACCEL的滤波器旁路控制配置。

    [2:0]为A_DLPF_CFG，此位为ACCEL的数字低通滤波器配置。

6. 有关低功耗配置

    详细请自行阅读手册。

### ODR配置总结
   
总的来说，要想配置自己心仪的ODR，可以分为以下几步

- 选择合适的量程
- 选择需要的输出速率
- 计算Simpe_Div

### 注意事项

  1. 有关于GYRO和ACCEL的ODR可以不同这件事

      虽然是Fs和Rate是两个功能几乎一样的东西，但是由于不同的选择Fs和Rate可能不一样且Simple_Div只有一个并且GYRO和ACCEL是通用一个(*没错，我也觉得很奇怪*)，就会导致它们分别的ODR可能是四倍或八倍等倍数关系，且一般情况下GYRO大于ACCEL。这个可能也是厂家的一些小设计吧，需要使用者根据情况去自行计算，精心配置。

  2. 关于Simple_Div的计算到底用谁去反算？

      Simple_Div只有一个并且GYRO和ACCEL是通用一个

      Simple_Div只有一个并且GYRO和ACCEL是通用一个

      Simple_Div只有一个并且GYRO和ACCEL是通用一个

      由于有这个限制，就导致计算这个东西让我感到很奇怪。网上其实都没有研究的这么细致过，只有自己仔细去读过手册，反复实验提问才能知道这些。

      大多数教程代码都是直接用GYRO去计算的，因为他们可能压根就不知道加速度计和陀螺仪的ODR其实是可以不一样的，都是使用的推荐的1KHz的内部采样率去计算的Simple_Div。

      ![ODR1](photos\ODR1.png)
      ![ODR2](photos\ODR2.png)

      确实通过表可以看出大多数时候加速度计和陀螺仪的内部采样率确实都是1KHz，此时Simple_Div通用完全没问题。

      但是从表里面我们也能看到有时候Fs或者Rate不一定是1KHz，也有高到32KHz的内部采样率。

      **Simple_Div_Fs = (Fs / ODR) - 1**
      **Simple_Div_Rate = (Rate / ODR) - 1**

      当目标ODR为100Hz，Fs为32KHz，Rate是1KHz时

      Simple_Div_Fs等于319，Simple_Div_Rate等于9

      Simple_Div_Fs甚至超出了Simple_Div最大值255，这肯定不行。

      将9带入GYRO的ODR，该ODR为3.2KHz，是目标的32倍，也有出入

      所以我认为只能这样理解————

      **任何时候都取内部采样率那个小的来计算采样率分频，且Fs/Rate就是陀螺仪和加速度计的输出数据速率之比**

      这样就可以解决这个Simple_Div选择的难题了。

      就拿刚才这个例子来说

      *当目标ODR为100Hz，Fs为32KHz，Rate是1KHz时*

      此时可以取Simple_Div为9，理解成陀螺仪输出速率是加速度计输出速率的32/1=32倍，且标准速率(这里就是加速度计的输出速率)为100Hz。

  3. 关于Simple_Div只能取某几个值的谬误

      以前在学习MPU6500的时候都会看到有个函数，大概就是某些ODR对应某个区间的值，这样就将Simple_Div锁定在某几个值上。

      我想应该是来源于这里

      ![ODR8](photos\ODR8.png)

      这个是加速度配置寄存器2下面的一句话，大概说的是在计算ODR公式下加速度计的输出速率可以配置出来的一个子集。很多人可能就理解成加速度计是不是只能输出这个子集里面的数据，然后就将Simple_Div取成只能计算出这个子集的系数。这是很严重的错误。

      实际上Simple_Div可以是0-255的任意数字，所以对应出来的加速度计ODR绝对不是只有这几个数的。相信你在看到前面的关于Simple_Div计算后应该也可以理解了。

      但是为什么官方的手册要写这个子集呢？

      这些值（3.91, 7.81... 1K）是一个示例性子集，目的是向读者展示，通过将 SMPLRT_DIV 设置为 2的幂次方减一 的值，可以得到一系列非常“整洁”的输出数据率 (ODR)。

      ![ODR9](photos\ODR9.png)

      其实再往下看看手册，就会发现其实这是和低功耗有关的，但是假如你不是特别要求这一块，Simple_Div对于你来说就是0-255。

## 有关FIFO

### FIFO简述

FIFO 的全称是 First-In, First-Out（先进先出）。你可以把它想象成 MPU6500 芯片内部的一个小型数据队列或传送带。

它的核心作用是 “缓冲” 和 “批处理”，主要为了解决一个关键问题：单片机读取速度跟不上传感器产生数据的速度。

根据电气手册，MPU6500 的FIFO**只有512字节**，而不是1024字节，与MPU6050或者MPU6000等老版本不一样。

**没有 FIFO 时（ polling / 轮询 ）：**

1. 传感器以固定的速率（例如ODR=1kHz）产生新数据。

2. 你的单片机必须以更快的速度（比如每0.9ms）不断地通过I2C/SPI去读取 GYRO_XOUT_H、GYRO_XOUT_L 等一大堆寄存器。

3. 一旦单片机因为执行其他任务（比如处理网络数据、复杂的数学计算）而稍微延迟了一点，就可能错过一组数据。因为新数据会覆盖旧数据寄存器。

4. 这会导致数据丢失，并且单片机一直处于忙碌的查询状态。

**有 FIFO 时：**

1. 传感器产生的新数据会自动、不停歇地被存入FIFO这个队列中。

2. 你的单片机可以安心地去处理其他任务，完全不用操心数据采集。

3. 当单片机忙完后，它可以一次性地从FIFO中读取多组数据（比如攒了10ms的10组数据）。

4. 因为数据被安全地存放在FIFO里，只要FIFO不满，就绝不会发生数据丢失。

### FIFO寄存器介绍

按照寄存器的先后顺序介绍

1. 配置寄存器（Register 26 – Configuration）

    ![FIFO1](photos\FIFO1.png)

    [6]FIFO_MODE，1为FIFO写满后新数据不再写入，0为写满后新数据覆盖最老的数据

2. FIFO使能（Register 35 – FIFO Enable）

    ![FIFO7](photos\FIFO7.png)

    [7:3]为开启温度，加速度，陀螺仪数据写入FIFO。

    每个数据分高位和低位，也就是两个字节。假如以上数据全开，也就是一组数据需要消耗2 x 7 = 14个字节。

3. 中断使能（Register 56 – Interrupt Enable）
   
    ![FIFO2_2](photos\FIFO2_2.png)
    ![FIFO2_1](photos\FIFO2_1.png)

    [4]使能FIFO溢出中断

4. 中断状态（Register 58 – Interrupt Status）

    ![FIFO3](photos\FIFO3.png)

    [4]FIFO溢出中断触发，此时假如你开启了FIFO覆盖，则触发中断时最旧的数据已经被覆盖了

5. FIFO计数器寄存器（Register 114 and 115 – FIFO Count Registers）

    ![FIFO4](photos\FIFO4.png)

    寄存器114的[4:0]为FIFO计数器的[12:8]位

    寄存器115的[7:0]为FIFO计数器的[7:0]位

    按道理来说0b1111_1111_1111应该是4095，总共能计数到4096个字节数据，但是由于FIFO只有512字节，所以高位应该用不到了。

    特别要说明，在读取这两个寄存器时，务必要先读高位，也就是114号寄存器，再读低位，也就是115号寄存器。因为必须要先读取高位才能将这两个寄存器锁存，否则会读取数据有误。

6. FIFO读写（Register 116 – FIFO Read Write）

    ![FIFO5_1](photos\FIFO5_1.png)
    ![FIFO5_2](photos\FIFO5_2.png)
    读取一个字节的数据

    需要连续读取该寄存器，也就是你可以设置一个读取长度，当FIFO计数器达到这个数的时候，你就连续读取这个长度的数据。

    假如还是一组数据14个字节，你想要读取10组数据取平均值作为一次有效数据，你就可以在FIFO计数器大于等于140时，连续读取该寄存器140次，此时就获得了这10组数据。这就是FIFO的底层逻辑。

7. 用户控制（Register 106 – User Control）

    ![FIFO6](photos\FIFO6.png)

    [6]FIFO_EN，FIFO的总开关

    [2]FIFO_RST，FIFO复位，清空FIFO的所有数据，在开启FIFO前必须先复位

    在读取到完整数据后，需要复位将老数据清空

    在FIFO溢出中断触发后，数据池可能被污染，此时需要复位

### FIFO配置流程

1. Register 106 – User Control复位FIFO
2. Register 106 – User Control开启FIFO
3. Register 26 – Configuration选择FIFO的写入模式
4. Register 35 – FIFO Enable选择要将哪些数据写入FIFO
5. Register 56 – Interrupt Enable选择是否开启FIFO溢出中断
6. Register 114 and 115 – FIFO Count Registers定时或者连续读取FIFO计数器来判断何时需要读取一波数据
7. Register 116 – FIFO Read Write读取数据
8. Register 106 – User Control再次复位清空数据

### 结尾

1. 特别注意MPU6500 的FIFO是512字节
2. MPU6050 或者其他型号的传感器似乎有FIFO水印配置寄存器
   
    可以设置这个寄存器来配置何时触发中断，也就是省去了在主循环中或者定时读取FIFO计数器这个步骤，当计数值达到水印值，就会自动触发中断，省时又给力。但是我找遍了MPU6500 的手册都没有找到这个寄存器，可能是被阉割了吧，有些可惜。

3. 读取状态

    **数据解析：**从FIFO读出的数据是按照你使能的顺序依次排列的字节流。例如，如果你使能了 ACCEL 和 XG|YG|ZG，那么FIFO中的数据顺序将是：ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L, GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L, GYRO_ZOUT_H, GYRO_ZOUT_L，然后如此循环。你需要根据这个顺序来解析。

    **字节序：**MPU6500的数据是大端序（Big-Endian），即高字节在前 (*_OUT_H)，低字节在后 (*_OUT_L)。

## 有关offset偏移寄存器

### 关于offset寄存器的简述

1. 


