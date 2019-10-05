# LTBL-ESC for STM32 (Lytsing Brushless ESC)
> 
The project contains LTBL Library designed for sensorless brushless motor.
> 
这是用于控制无感直流无刷电机的库
>
>
Features:
>
1.PWM & DSHOT600 throttle protocol supported and recognize automatically.
>
2.High electrical RPM. Up to 500,000 RPM.
>
3.Suitable for all kinds of BLDC. Applied to electric skateboard and quadcopter.
>
4.PWM mode is configurable, Synchronous rectification supported (High efficient).
>
5.Easy to work on your board. All peripherals & pins are defined in config file.
>
6.Rotate forward and reversely, brake supported.
>
7.More closed-loop function will be enabled after calibration of motor (For example: LinearBrake, Speed closed-loop mode, Force closed-loop mode).
>
特点：
>
1.支持 PWM / DSHOT600 油门协议，并可自动切换（示例用法在 DEMO 工程内）.
>
2.支持高转速电机，最高电转速 500,000 RPM.
>
3.适合各种直流无刷电机，既可应用于电动滑板（车）亦可应用于四轴飞行器.
>
4.PWM 模式可切换，支持同步整流模式（刹车回收能量，提升 PWM 效率）.
>
5.所有引脚 & 外设都已与控制代码解耦，可独立配置，以便运行在你的 PCB 上.
>
6.电机正转、刹车、倒车功能.
>
7.电机校准后可启用更多闭环功能（如：线性刹车、扭矩闭环、速度闭环）
>