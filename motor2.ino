#include "GyverPlanner2.h"
#include "EncButton.h"
Stepper< STEPPER2WIRE > stepperX(6, 7);
Stepper< STEPPER2WIRE > stepperY(8, 9);
GPlanner2< STEPPER2WIRE, 2 > planner;
Button GoNextButton(2);
EncButton TimeAdjEnc(4, 3, 5);
 
//время налива при включении 100 мс, текущие положения, сдвиг к первой точке
uint16_t TSpit = 100, XCurr = 0, YCurr = 0, XOrigin = 35, YOrigin = 90;
uint8_t XSteps = 5, YSteps = 3, XShift = 10, YShift = 10; //кол-во шагов и сдвиг
uint8_t XCount = 0, YCount = 0; // счетчики шагов 0 о умолч
int8_t XDir = 1, YDir = 1; //направление +1 или -1 
// массив координат для Planner, 0-0 по умолч. 
int path[1][2] = {
  {0, 0}
};

void setup() {
Serial.begin(115200);
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
GoNextButton.tick();
TimeAdjEnc.tick();
planner.tick();

//if (GoNextButton.press() && planner.ready()){
if (planner.ready()){
Spitter();
planner.resume();
}

if (planner.available()) {
path[0][0] = XCurr;
path[0][1] = YCurr;
planner.addTarget(path[0], 1);
MathinNext();
}



if (TimeAdjEnc.turn()){
TSpit += TimeAdjEnc.dir() * 50;
TSpit = (TSpit < 100) ? 100 : TSpit;
TSpit = (TSpit > 500) ? 500 : TSpit;
Serial.println(TimeAdjEnc.dir());
Serial.println(TSpit);
}
}

void Spitter() {
digitalWrite(13, HIGH);
static uint32_t Stmr;
Stmr = millis();
while (millis() - Stmr <= TSpit) {}
digitalWrite(13, LOW);
}

void MathinNext() {
++XCount;
if (XCount <= XSteps) {
XCurr = XCurr + XShift * XDir;
YCurr = YCurr;
}
if (XCount > XSteps) {
XCurr = XCurr;
YCurr = YCurr + YShift;
++YCount;
XDir = -XDir;
XCount = 0;
}
if (YCount <= YSteps) {
YCurr = YCurr;
}
if (YCount > YSteps) {
XCurr = 0;
YCurr = 0;
XDir = 1;
YDir = 1;
XCount = 0;
YCount = 0;
}
}
