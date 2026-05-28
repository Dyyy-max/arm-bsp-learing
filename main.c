/*****************************************
 * 项目：STM32电机PID闭环控制
 * 文件：main.c
 * 说明：大三嵌入式学习实践项目
 * 功能：系统初始化 + 主业务逻辑
 *****************************************/

#include <stdio.h>
#include <stdlib.h>
// STM32 HAL库总头文件
#include "stm32f1xx_hal.h"

UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim2;

// PID结构体定义
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float setpoint;
    float integral;
    float prev_error;
} PID_HandleTypeDef;

PID_HandleTypeDef motor_pid;
uint8_t motor_run = 0;
uint16_t pwm_duty = 500; // 初始占空比50%

// 串口printf重定向
int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}

// PID初始化
void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd, float setpoint) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->setpoint = setpoint;
    pid->integral = 0;
    pid->prev_error = 0;
}

// PID（计算增量式，带积分限幅）
float PID_Compute(PID_HandleTypeDef *pid, float measured) {
    float error = pid->setpoint - measured;
    // 1. 误差累加（积分项）+ 积分限幅，防止积分饱和
    pid->integral += error;
    if (pid->integral > 1000) pid->integral = 1000;
    if (pid->integral < -1000) pid->integral = -1000;

    // 2. 微分项
    float derivative = error - pid->prev_error;

    // 3. 计算PID输出
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    // 4. 限制输出范围（0-999，对应PWM占空比）
    if (output > 999) output = 999;
    if (output < 0) output = 0;

    // 5. 更新上一次误差
    pid->prev_error = error;

    return output;
}

// 系统时钟配置（CubeMX生成的标准模板）
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) while(1);
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) while(1);
}

// USART1初始化
static void MX_USART1_UART_Init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

// TIM2 PWM初始化
static void MX_TIM2_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    // 1. 定时器基础配置
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 71;          // 72MHz / (71+1) = 1MHz
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999;            // 1MHz / (999+1) = 1kHz PWM 频率
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    // 关键：开启自动重装载预装载，PWM更新无毛刺
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim2);

    // 2. 配置时钟源（内部时钟）
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

    // 3. 主模式配置（不需要触发信号，禁用主从模式）
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

    // 4. PWM通道配置（通道1）
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pwm_duty;          // 初始占空比，对应变量pwm_duty
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

    // 5. 启动定时器和PWM通道
    HAL_TIM_Base_Start(&htim2);          // 先启动定时器
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 再启动PWM通道
}

// GPIO初始化（按键、LED、电机使能）
static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // PA0 按键输入（上拉）
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PC13 LED输出
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // PB0 电机使能输出
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();

    // 初始化PID：Kp=2.0, Ki=0.1, Kd=0.05, 目标转速=800（模拟值）
    PID_Init(&motor_pid, 2.0f, 0.1f, 0.05f, 800.0f);

    printf("System Initialized!\r\n");
    while (1) {
       // 按键仿真：PA0低电平=按下，切换电机状态
       if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
           HAL_Delay(20); // 按下消抖
           if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
               motor_run = !motor_run;

               if (motor_run) {
                   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
                   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED亮
                   printf("Motor Started\r\n");
                } else {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED灭
                    printf("Motor Stopped\r\n");
        }

        // 等待按键松开，防止长按反复触发
        while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET);
    }
}

        if (motor_run) {
            // 无硬件仿真：用PWM值+随机波动模拟转速
            float simulated_speed = pwm_duty * 1.8f + (rand() % 100 - 50);
            
            // 限制转速范围（0-1500，避免出现负数或过大值）
            if (simulated_speed < 0) simulated_speed = 0;
            if (simulated_speed > 1500) simulated_speed = 1500;

            // PID计算新的PWM值
            pwm_duty = (uint16_t)PID_Compute(&motor_pid, simulated_speed);
            
            // 更新PWM占空比
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_duty);
            
            // 串口打印数据
            printf("Set:%.1f | Measured:%.1f | PWM:%d\r\n", 
                   motor_pid.setpoint, simulated_speed, pwm_duty);
        }

        HAL_Delay(100);
    }
}