#include <Arduino.h>
#include "FOCfunction.h"
#include "AS5600.h"

// 电机对象
Motor motor1(32, 33, 25, 7, 12.6f);

void setup()
{
  // 串口
  Serial.begin(115200);

  motor1.initPWM();
  motor1.initEncoder(19, 18);
  motor1.initVelPID(0.001, 0.1, 0, 0);
  motor1.initPosPID(0.133, 0, 0, 0);
  motor1.initCurPID(5, 200, 0, 100000);
  motor1.initCurSen(39, 36, 0.01f);

  Serial.println("完成初始化设置！");
}

float motor_target = 0;
int commaPosition;
String serialReceiveUserCommand()
{
  static String received_chars;
  String command = "";
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    received_chars += inChar;
    if (inChar == '\n')
    {
      // execute the user command
      command = received_chars;

      commaPosition = command.indexOf('\n'); // 检测字符串中的逗号
      if (commaPosition != -1)               // 如果有逗号存在就向下执行
      {
        motor_target = command.substring(0, commaPosition).toDouble(); // 电机角度
        Serial.println(motor_target);
      }
      received_chars = "";
    }
  }
  return command;
}

int i = 0;

void loop()
{
  
  ++i;
  if (i > 100)
  {
    Serial.println(motor1.getRealCurrent());
    i = 0;
  }
  
  motor1.currentLoop(motor_target);
  //motor1.velocityLoop(motor_target);
  serialReceiveUserCommand();
}