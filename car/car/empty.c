/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "board.h"
#include "bsp_tb6612.h"
// #include "math.h"
#include "stdlib.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <string.h>

/*motor struct*/

#define MOTOR_SPEED_RERATIO 20 // 电机减速比
#define PULSE_PRE_ROUND 13     // 每圈脉冲数
#define ENCODER_MULTIPLE 1.0   // 编码器倍频
#define PULSE_PER_CYCLE                                                        \
  (MOTOR_SPEED_RERATIO * PULSE_PRE_ROUND * ENCODER_MULTIPLE) // 每圈脉冲数
#define RADIUS_OF_TYRE 4.8                                   // 轮子直径 cm
#define LINE_SPEED_C RADIUS_OF_TYRE * 2 * 3.14               // 轮子周长 cm
#define SPEED_RECORD_NUM 20 // 速度记录值，用于均值滤波
#define WMA_WINDOW 5        // 加权滑动平均窗口大小
const float alpha = 0.8f;

typedef struct {
  uint8_t dierct;                       // 方向
  int32_t countnum;                     // 总计数值
  int32_t lastcount;                    // 上一次计数值
  float speed;                          // 电机速度
  float speed_Record[SPEED_RECORD_NUM]; // 记录电机近20次测速的结果
  // float iir_last;                       // IIR 上次输出
  // uint8_t iir_init;                     // IIR 初始化标志
  // float wma_buf[WMA_WINDOW];
  // uint8_t wma_idx;
} encoder_t;

// typedef struct {
//   uint8_t dierct;    // 方向
//   int32_t countnum;  // 总计数值
//   int32_t lastcount; // 上一次计数值
//   float speed;       // 电机速度
//   // float speed_Record[SPEED_RECORD_NUM];
//   /*—— 均值滤波（Moving Avg） ——*/
//   float ma_buf[SPEED_RECORD_NUM]; // 均值滤波专用环形缓存
//   uint8_t ma_idx;                 // 可选：圆环写指针（如果你改成圆环实现）

//   /*—— 中值滤波（Median） ——*/
//   float md_buf[SPEED_RECORD_NUM]; // 中值滤波专用缓存
//   uint8_t md_init;                // 首次填充标志

//   /*—— 加权滑动平均（WMA） ——*/
//   float wma_buf[WMA_WINDOW]; // WMA 缓存
//   uint8_t wma_idx;           // WMA 写指针

//   /*—— 指数平滑（IIR） ——*/
//   float iir_last;   // 上次输出
//   uint8_t iir_init; // 首次初始化标志
// } encoder_t;

typedef struct {
  float kp, ki, kd;             // pid参数
  float err, last_err;          // 误差，上一次误差
  float integral, max_integral; // 积分值，积分限幅
  float output, maxoutput;      // 输出，输出限幅
  float dead_zone;              // pid死区
} pid_t;

typedef struct {
  uint8_t dir; // 方向
  // uint8_t timer_index;  //PWM定时器通道
  int32_t pos_pulse; // 位置环目标脉冲
  // int32_t forward_GPIO_PIN; //前进GPIO引脚
  // int32_t reverse_GPIO_PIN;  //后退GPIO引脚
  float pwmDuty;     // PWM占空比
  float speed;       // 目标速度
  float pos;         // 目标位置
  encoder_t encoder; // 编码器
  pid_t pos_pid;     // 速度环pid
  pid_t speed_pid;   // 位置环pid
  // GPIO_Regs *forward_GPIO_PORT;  //前进GPIO端口
  // GPIO_Regs *reverse_GPIO_PORT;  // 后退GPIO端口
} motor_t;

/*motor struct*/

motor_t motor_l;
motor_t motor_r;
float route_l;
float route_r;
int8_t direction_l_last;
int8_t direction_r_last;
int8_t direction_l;
int8_t direction_r;

void motor_Init() {
  /*使能编码器输入捕获中断*/
  NVIC_EnableIRQ(ENCODER1A_INST_INT_IRQN);
  DL_TimerA_startCounter(ENCODER1A_INST);
  NVIC_EnableIRQ(ENCODER2A_INST_INT_IRQN);
  DL_TimerG_startCounter(ENCODER2A_INST);
  /*使能时钟定时器中断*/
  NVIC_EnableIRQ(CLOCK_INST_INT_IRQN);
  DL_TimerG_startCounter(CLOCK_INST);

  /*motor_l init*/
  motor_l.pwmDuty = 0;
  motor_l.speed = 0;
  //   motor_l.timer_index = DL_TIMER_CC_0_INDEX;
  //   motor_l.forward_GPIO_PORT = GPIO_MOTOR_AIN1_PORT;
  //   motor_l.reverse_GPIO_PORT = GPIO_MOTOR_AIN2_PORT;
  //   motor_l.forward_GPIO_PIN = GPIO_MOTOR_AIN1_PIN;
  //   motor_l.reverse_GPIO_PIN = GPIO_MOTOR_AIN2_PIN;
  motor_l.speed_pid.kp = -0;
  motor_l.speed_pid.ki = -0;
  motor_l.speed_pid.kd = 0;
  motor_l.speed_pid.max_integral = 10000;
  motor_l.speed_pid.maxoutput = 900;
  motor_l.speed_pid.dead_zone = 0;
  motor_l.speed_pid.output = 0;
  motor_l.pos_pid.kp = -0;
  motor_l.pos_pid.ki = 0;
  motor_l.pos_pid.kd = 0;
  motor_l.pos_pid.max_integral = 1000;
  motor_l.pos_pid.maxoutput = 999;
  motor_l.pos_pid.dead_zone = 0;
  motor_l.pos_pid.output = 0;

  /*motor_r init*/
  motor_r.pwmDuty = 0;
  motor_r.speed = 0;
  // motor_r.timer_index=DL_TIMER_CC_1_INDEX;
  // motor_r.forward_GPIO_PORT=GPIO_MOTOR_BIN1_PORT;
  // motor_r.reverse_GPIO_PORT=GPIO_MOTOR_BIN2_PORT;
  // motor_r.forward_GPIO_PIN=GPIO_MOTOR_BIN1_PIN;
  // motor_r.reverse_GPIO_PIN=GPIO_MOTOR_BIN2_PIN;
  motor_r.speed_pid.kp = -0;
  motor_r.speed_pid.ki = -0;
  motor_r.speed_pid.kd = 0;
  motor_r.speed_pid.max_integral = 10000;
  motor_r.speed_pid.maxoutput = 900;
  motor_r.speed_pid.dead_zone = 0;
  motor_r.speed_pid.output = 0;
  motor_r.pos_pid.kp = -0;
  motor_r.pos_pid.ki = 0;
  motor_r.pos_pid.kd = 0;
  motor_r.pos_pid.max_integral = 1000;
  motor_r.pos_pid.maxoutput = 999;
  motor_r.pos_pid.dead_zone = 0;
  motor_r.pos_pid.output = 0;

  // memset(motor_l.encoder.ma_buf, 0, sizeof motor_l.encoder.ma_buf);
  // memset(motor_l.encoder.md_buf, 0, sizeof motor_l.encoder.md_buf);
  // memset(motor_l.encoder.wma_buf, 0, sizeof motor_l.encoder.wma_buf);
  // motor_l.encoder.iir_init = 0;
  // motor_l.encoder.wma_idx = 0;
  // motor_l.encoder.md_init = 0;

  // memset(motor_r.encoder.ma_buf, 0, sizeof motor_r.encoder.ma_buf);
  // memset(motor_r.encoder.md_buf, 0, sizeof motor_r.encoder.md_buf);
  // memset(motor_r.encoder.wma_buf, 0, sizeof motor_r.encoder.wma_buf);
  // motor_r.encoder.iir_init = 0;
  // motor_r.encoder.wma_idx = 0;
  // motor_r.encoder.md_init = 0;
}

// 原来的均值滤波
float Speed_LLow_Filter(float new_Spe, encoder_t *encoder) // 均值滤波
{
  float sum = 0.0f;
  uint32_t test_Speed = new_Spe;
  for (uint8_t i = SPEED_RECORD_NUM - 1; i > 0; i--) {
    encoder->speed_Record[i] = encoder->speed_Record[i - 1];
    sum += encoder->speed_Record[i - 1];
  }
  encoder->speed_Record[0] = new_Spe;
  sum += new_Spe;
  test_Speed = sum / SPEED_RECORD_NUM;
  return sum / SPEED_RECORD_NUM;
}

// float Speed_LLow_Filter(float new_Spe, encoder_t *encoder) // 均值滤波
// {
//   float sum = 0.0f;
//   uint32_t test_Speed = new_Spe;
//   for (uint8_t i = SPEED_RECORD_NUM - 1; i > 0; i--) {
//     encoder->ma_buf[i] = encoder->ma_buf[i - 1];
//     sum += encoder->ma_buf[i - 1];
//   }
//   encoder->ma_buf[0] = new_Spe;
//   sum += new_Spe;
//   test_Speed = sum / SPEED_RECORD_NUM;
//   return sum / SPEED_RECORD_NUM;
// }

// // 其实是中值滤波
// float Speed_Low_Filter(float new_Spe, encoder_t *encoder) {
//   if (!encoder->md_init) {
//     // 第一次，全部填 new_Spe
//     for (int i = 0; i < SPEED_RECORD_NUM; i++)
//       encoder->md_buf[i] = new_Spe;
//     encoder->md_init = 1;
//     return new_Spe;
//   }
//   // 1. 更新环形缓冲区
//   //    将旧的值向后移动一位
//   for (uint8_t i = SPEED_RECORD_NUM - 1; i > 0; i--) {
//     encoder->md_buf[i] = encoder->md_buf[i - 1];
//   }
//   //    插入最新值
//   encoder->md_buf[0] = new_Spe;

//   // 2. 复制到临时数组并排序
//   float tmp[SPEED_RECORD_NUM];
//   memcpy(tmp, encoder->md_buf, sizeof(tmp));

//   // 简单冒泡排序（SPEED_RECORD_NUM 很小，可接受 O(N²)）
//   for (uint8_t i = 0; i < SPEED_RECORD_NUM - 1; i++) {
//     for (uint8_t j = i + 1; j < SPEED_RECORD_NUM; j++) {
//       if (tmp[i] > tmp[j]) {
//         float t = tmp[i];
//         tmp[i] = tmp[j];
//         tmp[j] = t;
//       }
//     }
//   }

//   // 3. 取中值
//   if (SPEED_RECORD_NUM % 2 == 1) {
//     // 奇数长度，直接返回中间那个
//     return tmp[SPEED_RECORD_NUM / 2];
//   } else {
//     // 偶数长度，取中间两个的均值
//     uint8_t m = SPEED_RECORD_NUM / 2;
//     return (tmp[m - 1] + tmp[m]) * 0.5f;
//   }
// }

// float Speed_ExpSmooth_Filter(float new_Spe, encoder_t *enc, float alpha) {
//   if (!enc->iir_init) {
//     enc->iir_last = new_Spe;
//     enc->iir_init = 1;
//   }
//   float y = alpha * enc->iir_last + (1.0f - alpha) * new_Spe;
//   enc->iir_last = y;
//   return y;
// }

// float Speed_WMA_Filter(float new_Spe, encoder_t *enc) {
//   // 1) 更新自己的 WMA 环形缓冲
//   enc->wma_buf[enc->wma_idx] = new_Spe;
//   if (++enc->wma_idx >= WMA_WINDOW)
//     enc->wma_idx = 0;

//   // 2) 计算加权平均
//   //    权重 = 1,2,...,WMA_WINDOW; 归一化后再求和
//   int sumW = WMA_WINDOW * (WMA_WINDOW + 1) / 2;
//   float y = 0;
//   for (int i = 0; i < WMA_WINDOW; i++) {
//     // idx 计算：i=0 对应最旧，i=WMA_WINDOW-1 对应最新
//     int idx = (enc->wma_idx + i) % WMA_WINDOW;
//     int w = i + 1;
//     y += w * enc->wma_buf[idx];
//   }
//   return y / sumW;
// }

float PID_Realize(pid_t *pid, float target, float feedback) {
  pid->err = target - feedback;
  if (pid->err < pid->dead_zone && pid->err > -pid->dead_zone)
    pid->err = 0; // pid死区
  pid->integral += pid->err;

  if (pid->ki * pid->integral < -pid->max_integral)
    pid->integral = -pid->max_integral / pid->ki; // 积分限幅
  else if (pid->ki * pid->integral > pid->max_integral)
    pid->integral = pid->max_integral / pid->ki;

  pid->output = (pid->kp * pid->err) + (pid->ki * pid->integral) +
                (pid->kd * (pid->err - pid->last_err));

  // 输出限幅
  if (pid->output > pid->maxoutput)
    pid->output = pid->maxoutput;
  else if (pid->output < -pid->maxoutput)
    pid->output = -pid->maxoutput;
  pid->last_err = pid->err;

  return pid->output;
}

// float abs(float x) { return x > 0 ? x : -x; }

int main(void) {
  SYSCFG_DL_init();

  DL_TimerA_enableInterrupt(
      ENCODER1A_INST,
      DL_TIMERA_INTERRUPT_CC0_UP_EVENT); // 使能捕获中断:contentReference[oaicite:9]{index=9}
  NVIC_EnableIRQ(
      ENCODER1A_INST_INT_IRQN); // NVIC
                                // 使能:contentReference[oaicite:10]{index=10}
  DL_TimerA_startCounter(
      ENCODER1A_INST); // 启动计数:contentReference[oaicite:11]{index=11}

  // 右侧编码器 (TimerG6 CC0 上升沿捕获)
  DL_TimerG_enableInterrupt(ENCODER2A_INST,
                            DL_TIMERG_INTERRUPT_CC0_UP_EVENT); // 使能捕获中断
  NVIC_EnableIRQ(ENCODER2A_INST_INT_IRQN);
  DL_TimerG_startCounter(ENCODER2A_INST);

  // 时钟定时器 (TIMG0 Zero Event)
  DL_TimerG_enableInterrupt(CLOCK_INST,
                            DL_TIMERG_INTERRUPT_ZERO_EVENT); // 使能零点中断
  NVIC_EnableIRQ(CLOCK_INST_INT_IRQN);
  DL_TimerG_startCounter(CLOCK_INST);

  TB6612_Motor_Stop();
  motor_Init();

  lc_printf("\nTB6612 Motor Demo Start...\r\n");

  while (1) {
    // 正反转自己定义

    // for (int i = 0; i < 1000; i += 50) {
    //   AO_Control(1, i); // A端电机转动 速度最大1000
    //   BO_Control(1, i); // B端电机转动 速度最大1000

    //   //   lc_printf("Dir[ 1 ]   i[ %d ]\r\n", i);

    //   delay_ms(500);
    // }
    // TB6612_Motor_Stop(); // 停止转动
    // delay_ms(1000);

    // for (int i = 0; i < 1000; i += 50) {
    //   AO_Control(0, i); // A端电机转动 速度最大1000
    //   BO_Control(0, i); // B端电机转动 速度最大1000

    //   //   lc_printf("Dir[ 0 ]   i[ %d ]\r\n", i);

    //   delay_ms(500);
    // }
    // TB6612_Motor_Stop(); // 停止转动
    // delay_ms(1000);

    // AO_Control(0, 90); // A端电机转动 速度最大1000
    // BO_Control(0, 90); // A端电机转动 速度最大1000
    // delay_ms(100);
    // lc_printf("%4d",
    //           DL_GPIO_readPins(GPIO_ENCODER_PORT,
    //           GPIO_ENCODER_ENCODER1B_PIN));
    // lc_printf("%4d",
    //           DL_GPIO_readPins(GPIO_ENCODER_PORT,
    //           GPIO_ENCODER_ENCODER2B_PIN));
  }
}

/*左电机的编码器测速*/
void TIMA0_IRQHandler(void) {
  switch (DL_TimerA_getPendingInterrupt(ENCODER1A_INST)) {
  case DL_TIMERA_IIDX_CC0_DN:
  DL_TimerA_clearInterruptStatus(ENCODER1A_INST, DL_TIMERA_IIDX_CC0_UP);
  motor_l.encoder.dierct = DL_GPIO_readPins(
      GPIO_ENCODER_PORT,
      GPIO_ENCODER_ENCODER1B_PIN); // 读取IO电平获取电机旋转方向
  motor_l.encoder.countnum =
      motor_l.encoder.dierct
          ? (motor_l.encoder.countnum + 1)
          : (motor_l.encoder.countnum -
             1); // 通过判断旋转方向来决定countnum增加还是减少
  // lc_printf("左编码器");
  break;
  default:
    break;
  }
}

/*右电机的编码器测速*/
void TIMG6_IRQHandler(void) {
  switch (DL_TimerG_getPendingInterrupt(ENCODER2A_INST)) {
  case DL_TIMERG_IIDX_CC0_DN:
  DL_TimerG_clearInterruptStatus(ENCODER2A_INST, DL_TIMERG_IIDX_CC0_UP);
  motor_r.encoder.dierct = DL_GPIO_readPins(
      GPIO_ENCODER_PORT,
      GPIO_ENCODER_ENCODER2B_PIN); // 读取IO电平获取电机旋转方向
  motor_r.encoder.countnum =
      motor_r.encoder.dierct
          ? (motor_r.encoder.countnum - 1)
          : (motor_r.encoder.countnum +
             1); // 通过判断旋转方向来决定countnum增加还是减少
  // lc_printf("右编码器");
    break;
  default:
    break;
  }


// lc_printf("%d,%d,%d,%d,%d,%d,%d,%d\r\n",DL_TIMERG_IIDX_CC0_DN,DL_TIMERG_IIDX_CC0_UP,DL_TIMERG_IIDX_CC1_DN,DL_TIMERG_IIDX_CC1_UP,DL_TIMERG_IIDX_CC2_DN,DL_TIMERG_IIDX_CC2_UP,DL_TIMERG_IIDX_CC3_DN,DL_TIMERG_IIDX_CC3_UP);

}

void TIMG0_IRQHandler(void) {
  switch (DL_TimerG_getPendingInterrupt(CLOCK_INST)) {
  case DL_TIMER_IIDX_ZERO:
    DL_TimerG_clearInterruptStatus(CLOCK_INST, DL_TIMER_IIDX_ZERO);
    motor_l.encoder.speed =
        (float)(motor_l.encoder.countnum - motor_l.encoder.lastcount) * 6000 /
        PULSE_PER_CYCLE; // rpm
    motor_r.encoder.speed =
        (float)(motor_r.encoder.countnum - motor_r.encoder.lastcount) * 6000 /
        PULSE_PER_CYCLE; // rpm
    // float raw_l = (motor_l.encoder.countnum - motor_l.encoder.lastcount) *
    //               6000.0f / PULSE_PER_CYCLE;
    // float raw_r = (motor_r.encoder.countnum - motor_r.encoder.lastcount) *
    //               6000.0f / PULSE_PER_CYCLE;
    // // 左电机
    // float m_l = Speed_LLow_Filter(raw_l, &motor_l.encoder); // 均值
    // // float md_l = Speed_Low_Filter(m_l, &motor_l.encoder);   // 中值
    // // float w_l = Speed_WMA_Filter(md_l, &motor_l.encoder);   // WMA
    // 用中值结果 motor_l.encoder.speed =
    //     Speed_ExpSmooth_Filter(raw_l, &motor_l.encoder, /*α=*/0.8f); //
    //     最后做
    // //     IIR

    // // 右电机
    // float m_r = Speed_LLow_Filter(raw_r, &motor_r.encoder);
    // // float md_r = Speed_Low_Filter(m_r, &motor_r.encoder);
    // // float w_r = Speed_WMA_Filter(md_r, &motor_r.encoder);
    // motor_r.encoder.speed =
    //     Speed_ExpSmooth_Filter(raw_r, &motor_r.encoder, /*α=*/0.8f);

    motor_l.encoder.speed =
        Speed_LLow_Filter(motor_l.encoder.speed, &motor_l.encoder);
    motor_r.encoder.speed =
        Speed_LLow_Filter(motor_r.encoder.speed, &motor_r.encoder);
    // motor_l.encoder.speed =
    //     Speed_Low_Filter(motor_l.encoder.speed, &motor_l.encoder);
    // motor_r.encoder.speed =
    //     Speed_Low_Filter(motor_r.encoder.speed, &motor_r.encoder);

    // // 2) 纯加权滑动平均
    // motor_l.encoder.speed =
    //     Speed_WMA_Filter(motor_l.encoder.speed, &motor_l.encoder);
    // motor_r.encoder.speed =
    //     Speed_WMA_Filter(motor_r.encoder.speed, &motor_l.encoder);
    // // 1) 纯指数平滑
    // motor_l.encoder.speed =
    //     Speed_ExpSmooth_Filter(motor_l.encoder.speed, &motor_l.encoder,
    //                            /*alpha=*/0.8f);
    // motor_r.encoder.speed =
    //     Speed_ExpSmooth_Filter(motor_r.encoder.speed, &motor_l.encoder,
    //                            /*alpha=*/0.8f);

    // // 3) 级联：先中值再指数平滑
    // {
    //   float m = Speed_Low_Filter(motor_l.encoder.speed, &motor_l.encoder);
    //   motor_l.encoder.speed =
    //       Speed_ExpSmooth_Filter(m, &motor_l.encoder, /*alpha=*/0.8f);
    // }
    // {
    //   float m = Speed_Low_Filter(motor_r.encoder.speed, &motor_l.encoder);
    //   motor_r.encoder.speed =
    //       Speed_ExpSmooth_Filter(m, &motor_l.encoder, /*alpha=*/0.8f);
    // }
    // // 4) 级联：先 WMA 再指数平滑
    // {
    //   float w = Speed_WMA_Filter(motor_l.encoder.speed, &motor_l.encoder);
    //   motor_l.encoder.speed =
    //       Speed_ExpSmooth_Filter(w, &motor_l.encoder, /*alpha=*/0.8f);
    // }
    // {
    //   float w = Speed_WMA_Filter(motor_r.encoder.speed, &motor_l.encoder);
    //   motor_r.encoder.speed =
    //       Speed_ExpSmooth_Filter(w, &motor_l.encoder, /*alpha=*/0.8f);
    // }
    motor_l.encoder.lastcount = motor_l.encoder.countnum;
    motor_r.encoder.lastcount = motor_r.encoder.countnum;
    // lc_printf("Dir[ l ]   i[ %f ]\r\n", motor_l.encoder.speed);
    // lc_printf("Dir[ r ]   i[ %f ]\r\n", motor_r.encoder.speed);
    // lc_printf("%f\n", motor_r.encoder.speed);
    route_l += 0.01 * motor_l.encoder.speed;
    route_r += 0.01 * motor_r.encoder.speed;
    PID_Realize(&(motor_l.speed_pid), 30, motor_l.encoder.speed);
    PID_Realize(&(motor_r.speed_pid), 30, motor_r.encoder.speed);
    static int8_t numes=0;
    numes++;
    if(numes == 10)
    lc_printf("d0:%f,%f,%f,%f,%f,%f\n", route_l, route_r, motor_l.encoder.speed,
              motor_r.encoder.speed, motor_l.speed_pid.output,
              motor_r.speed_pid.output),numes=0;
    if (motor_l.speed_pid.output > 0)
      direction_l = 1;
    else
      direction_l = 0;

    if (motor_r.speed_pid.output > 0)
      direction_r = 1;
    else
      direction_r = 0;

    if (direction_l ^ direction_l_last)
      TB6612_Motor_Stop();
    if (direction_r ^ direction_r_last)
      TB6612_Motor_Stop();
    BO_Control(direction_l, abs((int)motor_l.speed_pid.output) +
                                500); // A端电机转动速度最大1000
    AO_Control(direction_r, abs((int)motor_r.speed_pid.output) +
                                500); // B端电机转动速度最大1000
    direction_l_last = direction_l;
    direction_r_last = direction_r;
    break;
  default:
    break;
  }
}