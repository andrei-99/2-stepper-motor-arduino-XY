#include "GyverPlanner2.h"
#include "EncButton.h"
#include <AsyncStream.h>
#include <GParser.h>
Stepper< STEPPER2WIRE > stepperX(6, 7);
Stepper< STEPPER2WIRE > stepperY(8, 9);
GPlanner2< STEPPER2WIRE, 2 > planner;
Button GoNextButton(2);
EncButton TimeAdjEnc(4, 3, 5);
AsyncStream<100> Portal(&Serial, '\n', 20);

// Диапазоны управляющей команды
const int CMD_MIN = 0;
const int CMD_MAX = 99;

// Диапазоны координат
const int COORD_MIN = 0;
const int COORD_MAX = 1200;

// Диапазоны времени (мс)
const int TIME_MIN = 100;
const int TIME_MAX = 4000;
/* ===================== ДАННЫЕ ===================== */

int command = 0;
// массив координат для Planner, 0-0 по умолч.
int path[1][2] = {
  { 0, 0 }
};
int moveTime = 0;

//время налива при включении 100 мс, текущие положения, сдвиг к первой точке
uint16_t TSpit = 100, XCurr = 0, YCurr = 0, XOrigin = 35, YOrigin = 90;
uint8_t XSteps = 5, YSteps = 3, XShift = 10, YShift = 10;  //кол-во шагов и сдвиг
uint8_t XCount = 0, YCount = 0;                            // счетчики шагов 0 о умолч
int8_t XDir = 1, YDir = 1;                                 //направление +1 или -1

enum MachineState : uint8_t { IDLE,
                              PARSING,
                              MOVE,
                              SPIT };
MachineState currentState = MachineState::IDLE;

void setup() {
  Serial.begin(115200);
  Serial.println("Let's begin");

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  // добавляем шаговики на оси
  planner.addStepper(0, stepperX);  // ось 0
  planner.addStepper(1, stepperY);  // ось 1

  // устанавливаем ускорение и скорость
  planner.setAcceleration(1000);
  planner.setMaxSpeed(3000);
  // начальная точка системы должна совпадать с первой точкой маршрута
  //planner.setCurrent(path[0]);
  //planner.start();
  //planner.addTarget(path[0], 1);
}

void loop() {
  if (Portal.available()) {
    serialParsing();
  }
  //   //GoNextButton.tick();
  //   //TimeAdjEnc.tick();
  //   planner.tick();

  switch (currentState) {
    case IDLE:
      if (Portal.available()) {
        planner.tick();
        currentState = PARSING;
      }
      break;

    case PARSING:
      if (planner.available()) {
        planner.addTarget(path[0], 1);
        planner.tick();
      }
      currentState = MOVE;
      break;

    case MOVE:
      if (planner.ready()) {
        planner.tick();
        currentState = SPIT;
      }
      break;

    case SPIT:
      Spitter();
      planner.tick();
      planner.resume();
      currentState = IDLE;
      break;
  }
  //if (GoNextButton.press() && planner.ready()){
  // if (TimeAdjEnc.turn()){
  // TSpit += TimeAdjEnc.dir() * 50;
  // TSpit = (TSpit < 100) ? 100 : TSpit;
  // TSpit = (TSpit > 500) ? 500 : TSpit;
  // Serial.println(TimeAdjEnc.dir());
  // Serial.println(TSpit);
  // }
}


void serialParsing() {
  char* BufInput = Portal.buf;
  Serial.print("data: ");
  Serial.println(BufInput);
  // Проверка наличия символа конца пакета
  if (!strchr(BufInput, ';')) {
    Serial.println("ERROR: packet terminator ';' not found");
    return;
  }
  Serial.println("Step 2 >> GParser ");
  // Инициализация парсера
  GParser data2(BufInput, ',');

  // Проверка количества параметров
  Serial.print("parser.amount: ");
  Serial.println(data2.amount());
  if (data2.amount() != 4) {
    Serial.println("Packet length error");
    return;
  }
  // Парсинг
  int ints[data2.amount()];
  int am2 = data2.parseInts(ints);
  command = ints[0];
  path[0][0] = ints[1];
  path[0][1] = ints[2];
  moveTime = ints[3];

  // Валидация диапазонов
  if (command < CMD_MIN || command > CMD_MAX) {
    Serial.println("ERROR: command out of range");
    return;
  }

  if (path[0][0] < COORD_MIN || path[0][0] > COORD_MAX || path[0][1] < COORD_MIN || path[0][1] > COORD_MAX) {
    Serial.println("ERROR: coordinate out of range");
    return;
  }

  if (moveTime < TIME_MIN || moveTime > TIME_MAX) {
    Serial.println("ERROR: time out of range");
    return;
  }
  Serial.print("Data amount: ");
  Serial.println(data2.amount());
  // Если дошли сюда — пакет корректен
  Serial.print("CommandID: ");
  Serial.println(command);
  Serial.print("X: ");
  Serial.println(path[0][0]);
  Serial.print("Y: ");
  Serial.println(path[0][1]);
  Serial.print("ParameterID: ");
  Serial.println(moveTime);
}

void Spitter() {
  digitalWrite(13, HIGH);
  static uint32_t Stmr;
  Stmr = millis();
  while (millis() - Stmr <= TSpit) {}
  digitalWrite(13, LOW);
}

// void MathinNext() {
// ++XCount;
// if (XCount <= XSteps) {
// XCurr = XCurr + XShift * XDir;
// YCurr = YCurr;
// }
// if (XCount > XSteps) {
// XCurr = XCurr;
// YCurr = YCurr + YShift;
// ++YCount;
// XDir = -XDir;
// XCount = 0;
// }
// if (YCount <= YSteps) {
// YCurr = YCurr;
// }
// if (YCount > YSteps) {
// XCurr = 0;
// YCurr = 0;
// XDir = 1;
// YDir = 1;
// XCount = 0;
// YCount = 0;
// }
// }
