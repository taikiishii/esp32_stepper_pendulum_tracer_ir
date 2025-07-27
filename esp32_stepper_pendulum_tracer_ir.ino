#include <Wire.h>
#include <SSD1306.h>//ディスプレイ用ライブラリを読み込み
#include "fastestDigitalWrite.hpp"

// ピンアサイン
#define DIR_R 32
#define STEP_R 25
#define DIR_L 27
#define STEP_L 33
#define DIR_C 26
#define STEP_C 14
#define EN 13
#define SDA 21
#define SCL 22
#define LINE_R 35
#define LINE_L 34
#define BATTERY_MONITOR 4
#define IR_REMOTE 19

#define PULSE_PERIOD 20       // パルス生成の周期    20us
#define BALANCE_PERIOD 4000   // 倒立制御の周期       4ms
#define CONTROL_PERIOD 12000  // その他の制御の周期 12ms
#define LIMIT 450             // ステッピングモータが脱調しない最大のスピード マイクロステップ1/4=560
#define STEPS 480             // 1回転のSTEPS 1101:120 1332:480
#define MICRO_STEP 1          // ステッピングモータのマイクロステップの逆数 (1/4 -> 4)
#define WHEEL_DIAMETER 55

// 加速度・ジャイロセンサーの制御定数
#define MPU6050_ADDR 0x68        // MPU-6050 device address
#define MPU6050_SMPLRT_DIV 0x19  // MPU-6050 register address
#define MPU6050_CONFIG 0x1a
#define MPU6050_GYRO_CONFIG 0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_ACCEL_XOUT_H 0x3b
#define MPU6050_ACCEL_XOUT_L 0x3c
#define MPU6050_ACCEL_YOUT_H 0x3d
#define MPU6050_ACCEL_YOUT_L 0x3e
#define MPU6050_ACCEL_ZOUT_H 0x3f
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48
#define MPU6050_PWR_MGMT_1 0x6b
#define MPU6050_WHO_AM_I 0x75

// モード
#define MODE_STILL 0
#define MODE_REMOTE 1
#define MODE_TRACE 2
#define MODE_AUTO 3

// 赤外線リモコンのコマンド
// モード変更
#define CMD_MODE_CHANGE 216
// 静止モード
#define CMD_K1_UP 248 
#define CMD_K1_DOWN 88 
#define CMD_K2_UP 177 
#define CMD_K2_DOWN 33
#define CMD_K3_UP 16
#define CMD_K3_DOWN 128
#define CMD_K4_UP 17
#define CMD_K4_DOWN 129
// リモコンモード
#define CMD_FWD 160
#define CMD_STOP 32
#define CMD_BACK 0
#define CMD_LEFT 16
#define CMD_RIGHT 128
// ライントレースモード
#define CMD_KP_UP 248
#define CMD_KP_DOWN 88
#define CMD_SPEED_POSITIVE 160
#define CMD_SPEED_ZERO 32
#define CMD_SPEED_NEGATIVE 0
// 自動モード
#define CMD_CIRCLE_RIGHT 33
#define CMD_CIRCLE_RIGHT_BACK 129
#define CMD_CIRCLE_LEFT  177
#define CMD_CIRCLE_LEFT_BACK  17
#define CMD_SPIN_RIGHT 128
#define CMD_SPIN_LEFT 16
#define CMD_DIE 0

int lowBatteryCount = 0;

// 0:静止モード、1:リモコンモード、2:ライントレースモード、3:自動モード
volatile int mode = 0;

// 倒立振子の制御 k1:傾き　k2:倒れる速度　k3:位置　k4:移動速度
volatile float k1 = 300, k2 = 20, k3 = 50, k4 = 10;  //1101
// volatile float k1 = 300, k2 = 25, k3 = 50, k4 = 10;  //1101

// ライントレースの制御 kp:
volatile float kp = 0.01;

// timer1はステッピングモータを動かすパルス生成用、timer2は倒立制御用、timer3はその他の処理
hw_timer_t* timer1 = NULL;
hw_timer_t* timer2 = NULL;
hw_timer_t* timer3 = NULL;
volatile SemaphoreHandle_t timerSemaphore1;
volatile SemaphoreHandle_t timerSemaphore2;
volatile SemaphoreHandle_t timerSemaphore3;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux2 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux3 = portMUX_INITIALIZER_UNLOCKED;


volatile bool outL = false, outR = false;
volatile long countL = 0, countR = 0;
volatile long controlSpeed = 0;
volatile long motorL = 0, motorR = 0;

volatile int16_t rawGyroY;
volatile int32_t lastUptime, startTime, currentTime;

volatile float dt;
volatile float caribGyroY;
volatile float gyroY, degY = 0, dpsY = 0;
volatile float lpfY, lpfA = 0.999;
volatile float pos = 0;
volatile int batteryMonitor;

volatile float moveSpeed = 0;
volatile float turnSpeed = 0;
volatile float turnSpeedL = 0;
volatile float turnSpeedR = 0;
volatile int thresholdWhite = 0;
volatile int thresholdBlack = 0;

// ライントレース用
volatile int initialLineValueL = 0;
volatile int initialLineValueR = 0;
volatile int lineValueL = 0;
volatile int lineValueR = 0;

// IRリモコン用
volatile int rm_state = 0;
volatile unsigned long prev_micros = 0;
volatile int digit = 0;
volatile unsigned long rm_code = 0;
volatile bool rm_received = false;

// OELDディスプレイ
SSD1306  display(0x3c, 21, 22); //SSD1306インスタンスの作成（I2Cアドレス,SDA,SCL）
char displayBuffer[32];  //16ドットフォントだと最大8文字

// センサーへのコマンド送信
void writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// センサーからのデータ読み込み
byte readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 1, true);
  while (!Wire.available()) {
    //Serial.println("Waiting for Wire.available（）");
  }
  byte data = Wire.read();
  Wire.endTransmission(true);
  return data;
}

unsigned long get_data_code() {
  rm_received = false;  // 初期化
  rm_state = 0;         // 初期化
  // 下16bitがcustomCode
  unsigned long custom_code = rm_code & 0xffff;
  // 下16bitを捨てた後の下8bitがdataCode
  unsigned long data_code = (rm_code & 0xff0000) >> 16;
  // 下24bitを捨てた後の下8bitがinvDataCode
  unsigned long inv_data_code = (rm_code & 0xff000000) >> 24;
  // エラー確認
  if ((data_code + inv_data_code) != 0xff) {
    data_code = 255;  // エラー
  }
  Serial.print("data_code=");
  Serial.println(data_code);
  display.init();
  sprintf(displayBuffer, "%d", data_code);
  display.drawString(0, 0, displayBuffer);
  display.display();
  return data_code;
}

void change_mode() {
  mode += 1;
  if (mode == 4) {
    mode = 0;
  }

  display.init();
  if (mode == MODE_STILL) {
    display.drawString(0, 0, "STILL MODE");
    moveSpeed = 0;
    turnSpeed = 0;
    turnSpeedL = 0;
    turnSpeedR = 0;
  } else if (mode == MODE_REMOTE) {
    display.drawString(0, 0, "REMOTE MODE");
    moveSpeed = 0;
    turnSpeed = 0;
    turnSpeedL = 0;
    turnSpeedR = 0;
  } else if (mode == MODE_TRACE) {
    display.drawString(0, 0, "TRACE MODE");
    moveSpeed = 0.3;
    turnSpeed = 20;
    // ラインの初期値を取得
    initialLineValueL = analogRead(LINE_L);
    initialLineValueR = analogRead(LINE_R);
    Serial.print("Line_L:");
    Serial.print(initialLineValueL);
    Serial.print("Line_R:");
    Serial.println(initialLineValueR);
    thresholdWhite = 3200;
    thresholdBlack = 2200;

  } else if (mode == MODE_AUTO) {
    display.drawString(0, 0, "AUTO MODE");
    moveSpeed = 0;
    turnSpeed = 0;
    turnSpeedL = 0;
    turnSpeedR = 0;
  }
  display.display();
}

// ステッピングモーターを駆動
void IRAM_ATTR onTimer1() {
  countL += motorL * MICRO_STEP * STEPS / 480;
  if (countL > 5000) {
    fastestDigitalWrite(STEP_L, outL);
    outL = !outL;
    countL -= 5000;
  } else if (countL < -5000) {
    fastestDigitalWrite(STEP_L, outL);
    outL = !outL;
    countL += 5000;
  }

  countR += motorR * MICRO_STEP * STEPS / 480;
  if (countR > 5000) {
    fastestDigitalWrite(STEP_R, outR);
    outR = !outR;
    countR -= 5000;
  } else if (countR < -5000) {
    fastestDigitalWrite(STEP_R, outR);
    outR = !outR;
    countR += 5000;
  }
}

// 倒立制御
void IRAM_ATTR onTimer2() {
  xSemaphoreGiveFromISR(timerSemaphore2, NULL);
}

// その他の処理
void IRAM_ATTR onTimer3() {
  xSemaphoreGiveFromISR(timerSemaphore3, NULL);
}

// リモコン信号の受信を処理
void IRAM_ATTR int_handler() {
    unsigned long width;
    if (rm_state != 0) {
        // 前回の信号変化からの時間間隔を計算
        width = micros() - prev_micros;
        if (width > 10000) { // 長すぎ
            rm_state = 0; // データ棄却
        }
        prev_micros = micros();
    }

    if (rm_state == 0) {
        prev_micros = micros();
        rm_state = 1;
        rm_code = 0;
        digit = 0;
    } else if (rm_state == 1) {
        if (width > 9500 || width < 8500) {
            rm_state = 0;
        } else {
            rm_state = 2;
        }
    } else if (rm_state == 2) {
        if (width > 5000 || width < 4000) {
            rm_state = 0;
        } else {
            rm_state = 3;
        }
    } else if (rm_state == 3) {
        if (width > 700 || width < 400) {
            rm_state = 0;
        } else {
            rm_state = 4;
        }
    } else if (rm_state == 4) {
        if (width > 1800 || width < 400) {
            rm_state = 0;
        } else {
            if (width > 1000) {
                rm_code |= (1 << digit);
            } else {
                rm_code &= ~(1 << digit);
            }
            digit++;
            if (digit > 31) {
                rm_received = true;
                return;
            }
            rm_state = 3;
        }
    }
}

void loop_auto(unsigned long data_code) {
  
  pos -= moveSpeed;

  switch (data_code) {
    case CMD_CIRCLE_RIGHT:
      moveSpeed = 0.5;
      turnSpeedL = 50;
      turnSpeedR = 0;
      displayAutoParameter();
      break;
    case CMD_CIRCLE_RIGHT_BACK:
      moveSpeed = -0.5;
      turnSpeedL = -50;
      turnSpeedR = 0;
      displayAutoParameter();
      break;
    case CMD_CIRCLE_LEFT:
      moveSpeed = 0.5;
      turnSpeedL = 0;
      turnSpeedR = 50;
      displayAutoParameter();
      break;
    case CMD_CIRCLE_LEFT_BACK:
      moveSpeed = - 0.5;
      turnSpeedL = 0;
      turnSpeedR = - 50;
      displayAutoParameter();
      break;
    case CMD_SPIN_RIGHT:
      moveSpeed = 0;
      turnSpeedL = 50;
      turnSpeedR = -50;
      displayAutoParameter();
      break;
    case CMD_SPIN_LEFT:
      moveSpeed = 0;
      turnSpeedL = -50;
      turnSpeedR = 50;
      displayAutoParameter();
      break;
    case CMD_STOP:
      moveSpeed = 0;
      turnSpeedL = 0;
      turnSpeedR = 0;
      displayAutoParameter();
      break;
    case CMD_DIE:
      k1=0;
      k2=0;
      k3=0;
      k4=0;
      moveSpeed = 0;
      turnSpeedL = 100;
      turnSpeedR = -100;
      displayAutoParameter();
      break;
    default: break;
  }
}

void displayAutoParameter() {
  display.init();
  sprintf(displayBuffer, "Auto");
  display.drawString(0, 0, displayBuffer);
  display.display();
}

void loop_remote(unsigned long data_code) {
  
  pos -= moveSpeed;

  switch (data_code) {
    case CMD_FWD:
      moveSpeed += 0.1;
      displayRemoteParameter();
      break;
    case CMD_STOP:
      moveSpeed = 0;
      displayRemoteParameter();
      break;
    case CMD_BACK:
      moveSpeed -= 0.1;
      displayRemoteParameter();
      break;
    case CMD_LEFT:
      turnSpeedL += 5;
      turnSpeedR -= 5;
      displayRemoteParameter();
      break;
    case CMD_RIGHT:
      turnSpeedL -= 5;
      turnSpeedR += 5;
      displayRemoteParameter();
      break;
    default: break;
  }
}

void displayRemoteParameter() {
  display.init();
  sprintf(displayBuffer, "MoveSpeed: %.4f", moveSpeed);
  display.drawString(0, 0, displayBuffer);
  sprintf(displayBuffer, "TurnLeft: %f", turnSpeedL);
  display.drawString(0, 16, displayBuffer);
  sprintf(displayBuffer, "TurnRight: %f", turnSpeedR);
  display.drawString(0, 32, displayBuffer);
  display.display();
}

void loop_still(unsigned long data_code) {
  switch (data_code) {
    case CMD_K1_UP: k1 *= 1.1; displayPidParameter(); break;
    case CMD_K1_DOWN: k1 *= 0.9090909; displayPidParameter(); break;
    case CMD_K2_UP: k2 *= 1.1; displayPidParameter(); break;
    case CMD_K2_DOWN: k2 *= 0.9090909; displayPidParameter(); break;
    case CMD_K3_UP: k3 *= 1.1; displayPidParameter(); break;
    case CMD_K3_DOWN: k3 *= 0.9090909; displayPidParameter(); break;
    case CMD_K4_UP: k4 *= 1.1; displayPidParameter(); break;
    case CMD_K4_DOWN: k4 *= 0.9090909; displayPidParameter(); break;
    default: break;
  }
}

void displayPidParameter() {
  display.init();
  sprintf(displayBuffer, "K1: %.4f", k1);
  display.drawString(0, 0, displayBuffer);
  sprintf(displayBuffer, "K2: %.4f", k2);
  display.drawString(0, 16, displayBuffer);
  sprintf(displayBuffer, "K3: %.4f", k3);
  display.drawString(0, 32, displayBuffer);
  sprintf(displayBuffer, "K4: %.4f", k4);
  display.drawString(0, 48, displayBuffer);
  display.display();
}

void loop_trace(unsigned long data_code) {
  lineValueL = analogRead(LINE_L);
  lineValueR = analogRead(LINE_R);

  // P(比例)制御
  turnSpeedL = (lineValueL - initialLineValueL) * kp;  //左が明るくなったら左を進めて右折
  turnSpeedR = (lineValueR - initialLineValueR) * kp;  //右が明るくなったら右を進めて左折

  // ON-OFF制御 エッジトレース
  // if (lineValueL < thresholdBlack && lineValueR < thresholdBlack) {  //Lが黒、Rが黒ならなら左折
  //   turnSpeedL = 0 - turnSpeed;
  //   turnSpeedR = turnSpeed;
  // } else if (lineValueL > thresholdWhite && lineValueR > thresholdWhite) {  // Lが白、Rが白ならなら右折
  //   turnSpeedL = turnSpeed;
  //   turnSpeedR = 0 - turnSpeed;
  // } else {  // それ以外なら直進
  //   ;
  // }

  pos -= moveSpeed;

  switch (data_code) {
    case CMD_KP_UP:
      kp *= 1.1;
      displayTraceParameter();
      break;
    case CMD_KP_DOWN:
      kp *= 0.9090909;
      displayTraceParameter();
      break;
    case CMD_SPEED_POSITIVE:
      if (moveSpeed < 1) {
        moveSpeed += 0.1;
      }
      displayTraceParameter();
      break;
    case CMD_SPEED_NEGATIVE:
      if (moveSpeed > -1) {
        moveSpeed -= 0.1;
      }
      displayTraceParameter();
      break;
    case CMD_SPEED_ZERO:
      moveSpeed = 0;
      displayTraceParameter();
      break;
    default: break;
  }
}


void displayTraceParameter() {
  display.init();
  sprintf(displayBuffer, "Speed: %.4f", moveSpeed);
  display.drawString(0, 0, displayBuffer);
  sprintf(displayBuffer, "Kp: %.4f", kp);
  display.drawString(0, 16, displayBuffer);
  display.display();
}

void setup() {
  Serial.begin(115200);
  Serial.println("*******************RESTARTED********************");
  Serial.println("*****************stepper_pendulum***************");
  Serial.print(k1); Serial.print(","); Serial.print(k2); Serial.print(","); Serial.print(k3); Serial.print(","); Serial.print(k4); Serial.println();

  // OLEDディスプレイ
  display.setFont(ArialMT_Plain_16);    //フォントを設定 サイズは10, 16, 24
  display.init(); 
  display.drawString(0, 0, "StepperPendulum");
  display.display();

  // モータードライバ制御用ピンの初期化
  pinMode(DIR_L, OUTPUT);
  pinMode(STEP_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(STEP_R, OUTPUT);
  pinMode(DIR_C, OUTPUT);
  pinMode(STEP_C, OUTPUT);
  pinMode(EN, OUTPUT);


  // I2Cの初期化
  Wire.setClock(400000);
  Wire.begin(SDA, SCL);

  // ジャイロセンサーの初期化
  if (readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
    Serial.println("WHO_AM_I error.");
    while (true)
      ;
  } else {
    Serial.println("WHO_AM_I OK.");
  }

  writeMPU6050(MPU6050_SMPLRT_DIV, 0x07);    // sample rate: 8kHz/(7+1) = 1kHz
  writeMPU6050(MPU6050_CONFIG, 0x00);        // disable DLPF, gyro output rate = 8kHz
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x00);   // gyro range: 0x00⇒±250dps 131LSB、0x08⇒±500dps 65.5LSB、0x10⇒±1000dps 32.8LSB、0x18⇒±2000dps 16.4LSB
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);  // accel range: 0x00⇒±2g 16384LSB/g、0x01⇒±4g 8192LSB/g、0x02⇒±8g 4096LSB/g、0x03⇒±16g 2048LSB/g、
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);    // disable sleep mode, PLL with X gyro
  Serial.println("MPU6050 Setup OK.");
  delay(2000);

  // ジャイロのゼロ点調整のために静止時の出力を1000回計測して平均を算出
  caribGyroY = 0;
  for (int i = 0; i < 1000; i++) {
    rawGyroY = (readMPU6050(MPU6050_GYRO_YOUT_H) << 8) | readMPU6050(MPU6050_GYRO_YOUT_L);
    caribGyroY += (float)rawGyroY;
  }
  caribGyroY /= 1000;
  Serial.print("Carib OK. caribGyroY:");
  Serial.println(caribGyroY);

  // 倒立時間計測用
  startTime = micros();

  // dt計測用
  lastUptime = micros();

  // ステッピングモータを有効化
  digitalWrite(EN, LOW);

  // 割込みタイマの設定
  timerSemaphore1 = xSemaphoreCreateBinary();
  timer1 = timerBegin(0, getApbFrequency() / 1000000, true);  // timer=1us
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, PULSE_PERIOD, true);
  timerAlarmEnable(timer1);

  timerSemaphore2 = xSemaphoreCreateBinary();
  timer2 = timerBegin(1, getApbFrequency() / 1000000, true);  // timer=1us
  timerAttachInterrupt(timer2, &onTimer2, true);
  timerAlarmWrite(timer2, BALANCE_PERIOD, true);
  timerAlarmEnable(timer2);

  timerSemaphore3 = xSemaphoreCreateBinary();
  timer3 = timerBegin(2, getApbFrequency() / 1000000, true);  // timer=1us
  timerAttachInterrupt(timer3, &onTimer3, true);
  timerAlarmWrite(timer3, CONTROL_PERIOD, true);
  timerAlarmEnable(timer3);

	// リモコン用
  pinMode(IR_REMOTE, INPUT_PULLUP);
	attachInterrupt(IR_REMOTE, int_handler, CHANGE);  // GPIO割り込みを設定

  // GO!
  display.init();
  Serial.println("GO!");
  display.drawString(0, 0, "GO!");
  display.display();
}

void loop() {
  if (xSemaphoreTake(timerSemaphore2, 0) == pdTRUE) {
    // dt計測
    currentTime = micros();
    dt = (currentTime - lastUptime) * 0.000001;
    lastUptime = currentTime;

    // 角速度を取得
    rawGyroY = (readMPU6050(MPU6050_GYRO_YOUT_H) << 8) | readMPU6050(MPU6050_GYRO_YOUT_L);
    gyroY = (float)rawGyroY - caribGyroY;
    dpsY = gyroY / 131.0;

    // 角速度を積算して角度を求める
    degY += dpsY * dt;

    Serial.println(degY);

    //ローパスフィルタでドリフトを補正
    lpfY *= lpfA;
    lpfY += (1 - lpfA) * degY;

    // スピードを積算して位置を求める
    pos += controlSpeed * dt;

    // 操作量の計算
    controlSpeed += (k1 * (degY - lpfY) + k2 * dpsY + k3 * pos + k4 * controlSpeed) * dt;

    //倒立制御 + 移動/転回
    motorL = controlSpeed + turnSpeedL;
    motorR = controlSpeed + turnSpeedR;

    motorL = constrain(motorL, 0 - LIMIT * MICRO_STEP, LIMIT * MICRO_STEP);
    motorR = constrain(motorR, 0 - LIMIT * MICRO_STEP, LIMIT * MICRO_STEP);

    // ステッピングモータの回転方向を指定
    digitalWrite(DIR_L, (motorL < 0));
    digitalWrite(DIR_R, (motorR < 0));

    // 倒れたらモーター停止
    if (45 < abs(degY - lpfY)) {
      controlSpeed = 0;
      digitalWrite(EN, HIGH);
      Serial.println("*********************STOP***********************");
      Serial.println((lastUptime - startTime) / 1000000);
      display.init();
      display.drawString(0, 0, "STOP");
      sprintf(displayBuffer, "%d", (lastUptime - startTime) / 1000000);
      display.drawString(48, 0, displayBuffer);
      display.display();
      while (true)
        ;
    }
  }

  if (xSemaphoreTake(timerSemaphore3, 0) == pdTRUE) {
    unsigned long data_code = 255;  // 255は何もしない
    // リモコンで受信した場合
    if (rm_received) {
      data_code = get_data_code();
      if (data_code == CMD_MODE_CHANGE) {
        change_mode();
      }
    }

    switch (mode) {
      case MODE_STILL:
        loop_still(data_code);
        break;
      case MODE_REMOTE:
        loop_remote(data_code);
        break;
      case MODE_TRACE:
        loop_trace(data_code);
        break;
      case MODE_AUTO:
        loop_auto(data_code);
        break;
      default: break;
    }

    // バッテリ電圧の監視
    batteryMonitor = analogReadMilliVolts(BATTERY_MONITOR) * 4;  // 1/4に分圧している
    if (batteryMonitor < 3000) {                                 //閾値は3.0V
      lowBatteryCount += 1;
      if (lowBatteryCount > 5) {  // 5回連続で閾値を下回ったら停止
        controlSpeed = 0;
        digitalWrite(EN, HIGH);
        Serial.print("LOW BATTERY.");
        Serial.println(batteryMonitor);
        sprintf(displayBuffer, "%d", batteryMonitor);
        display.init();
        display.drawString(0, 0, "LOW BATTERY");
        display.drawString(48, 0, displayBuffer);
        display.display();
        while (true) {
          batteryMonitor = analogReadMilliVolts(BATTERY_MONITOR) * 4;
          vTaskDelay(1000 / portTICK_RATE_MS);
        }
      } else {
        lowBatteryCount = 0;
      }
    }
  }
}
