#include "Servo.h"
#include <Wire.h>
#include <Arduino.h>
#include "kren_ctrl.hpp"
#include "MPU6050.h"
#include "I2Cdev.h"
#define right_motor 9
#define left_motor 3
#include <EEPROM.h>
Servo Motor1;
Servo Motor2;
MPU6050 mpu;
#define BUFFER_SIZE 100
#define START_BYTE 1010
int16_t ax, ay, az;
int16_t gx, gy, gz;
bool calibrated = false;  // Флаг для отслеживания состояния калибровки

String inputString = "";      // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
bool manualCmd = false;
KrenMdl mdl = KrenMdl();
KrenCtrl ctrl = KrenCtrl();
float setFi = 0;
uint16_t Upwr = 0;
uint16_t sU1 = 0;
uint16_t sU2 = 0;
float Acc_filter =0;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  mpu.initialize();
  int offsets[6];
  EEPROM.get(START_BYTE, offsets);
    // Применяем загруженные оффсеты
  mpu.setXAccelOffset(offsets[0]);
  mpu.setYAccelOffset(offsets[1]);
  mpu.setZAccelOffset(offsets[2]);
  mpu.setXGyroOffset(offsets[3]);
  mpu.setYGyroOffset(offsets[4]);
  mpu.setZGyroOffset(offsets[5]);

  Serial.println(F("Send any character to recalibrate MPU6050"));
  Motor1.attach(right_motor);
  Motor2.attach(left_motor);
  inputString.reserve(20);
}

uint8_t gLog = false;

const char* nms [] = {
"stf", //Set angle
"Kpv", // Parameter P of velocity contur 
"Kdv", // Parameter D of velocity contur   
"Tmi", // Parameter of intergrant for saturate uI
"Tiv", // Parameter I of velocity contur for apply uI
"Tfv", // Parameter of demph for real diff
"Kpf", // Parameter P of anglular contur
"Kdf", // Parameter D of anglular contur
"Tif", // Parameter for I of intergrant for angular contur
"Tff", // Parameter for I of intergrant after velocity 
"Tfr", // Parameter Tf of regulator 
"Tvr", // Parameter Tv of regulator
"Tor", // Parameter To of regulator
"Tfm", // Parameter Tf of model
"Tvm", // Parameter Tv of model
"Tom", // Parameter To of model
"Ctp", // Control type:  enum { eManual=0, eP_Pid, eP_PDI, eSlide, eMax}
"Klm", // Parameter for Kalman
"stU", // Set U1 U2 in manual mode
"sUm", // Set Um in manual mode
"clP", // Calculate opt parameters from object param
"sU1", // Set U1 in manual mode
"sU2", // Set U2 in manual mode
"" };

enum idPrm { e_stf=0, e_Kpv, e_Kdv, e_Tmi, e_Tiv, e_Tfv, e_Kpf, e_Kdf, e_Tif, e_Tff, 
              e_Tfr, e_Tvr, e_Tor, e_Tfm, e_Tvm, e_Tom, e_Ctp, e_Klm, e_stU, e_sUm, e_clp, e_sU1, e_sU2, e_PrmSz };

void parse(String str){
  if (str == "al2") { gLog = 2; return; }
  if (str == "al3") { gLog = 3; return; }
  if (str == "al4") { gLog = 4; return; }
  if (str == "al5") { gLog = 5; return; }
  if (str == "al") { gLog = true; return; }
  if (str == "an") { gLog = false; return; }
  if (str == "ac") { ctrl.setCtrlParam(); return; }
  if (str.substring(0,3) == "as:"){
    if (str.substring(3,6) == nms[e_stf]) {  setFi = 1.0 * str.substring(7).toInt(); return; }
    if (str.substring(3,6) == nms[e_Kpv]) {  ctrl.Kpv = 0.00001 * str.substring(7).toInt(); return; }
    if (str.substring(3,6) == nms[e_Kdv]) {  ctrl.Kdv = 0.00001 * str.substring(7).toInt(); return; }
    if (str.substring(3,6) == nms[e_Tmi]) {  ctrl.Tmi = 0.001 * str.substring(7).toInt(); return; }
    if (str.substring(3,6) == nms[e_Tiv]) {  ctrl.Tiv = 0.001 * str.substring(7).toInt(); return; }
    if (str.substring(3,6) == nms[e_Tfv]) {  ctrl.Tfv = 0.001 * str.substring(7).toInt(); return; }
    if (str.substring(3,6) == nms[e_Kpf]) {  ctrl.Kpf = 0.001 * str.substring(7).toInt(); return; }
    if (str.substring(3,6) == nms[e_Kdf]) {  ctrl.Kdf = 0.001 * str.substring(7).toInt(); return; }
    if (str.substring(3,6) == nms[e_Tif]) {  ctrl.Tif = 0.001 * str.substring(7).toInt(); return; }
    if (str.substring(3,6) == nms[e_Tff]) {  ctrl.Tff = 0.001 * str.substring(7).toInt(); return; }
    if (str.substring(3,6) == nms[e_Tfr]) {  ctrl.Tfm = 0.001 * str.substring(7).toInt(); return; }
    if (str.substring(3,6) == nms[e_Tfm]) {  mdl.Tfm = 0.001 * str.substring(7).toInt(); return; }
    if (str.substring(3,6) == nms[e_Tvr]) {  ctrl.Tvm = 0.001 * str.substring(7).toInt(); return; }
    if (str.substring(3,6) == nms[e_Tvm]) {  mdl.Tvm = 0.001 * str.substring(7).toInt(); return; }
    if (str.substring(3,6) == nms[e_Tor]) {  ctrl.Tom = 0.001 * str.substring(7).toInt(); return; }
    if (str.substring(3,6) == nms[e_Tom]) {  mdl.Tom = 0.001 * str.substring(7).toInt(); return; }
    if (str.substring(3,6) == nms[e_Ctp]) {  
      ctrl.CtrlType = str.substring(7).toInt(); 
      if(ctrl.CtrlType == 0) { Upwr = 1000; ctrl.uM = 0; }
      return; 
    }
    if (str.substring(3,6) == nms[e_Klm]) {  ctrl.Klm = 0.001 * str.substring(7).toInt(); return; }
    if (str.substring(3,6) == nms[e_stU]) {  Upwr = str.substring(7).toInt(); manualCmd = true; return; }
    if (str.substring(3,6) == nms[e_sUm]) {  ctrl.uM = 1.0 * str.substring(7).toInt(); manualCmd = true; return; }
    if (str.substring(3,6) == nms[e_sU1]) {  sU1 = str.substring(7).toInt(); return; }
    if (str.substring(3,6) == nms[e_sU2]) {  sU2 = str.substring(7).toInt(); return; }
  }
  if (str.substring(0,3) == "ag:"){
    gLog = false;
    int16_t rslt = -1;    
    if (str.substring(3,6) == nms[e_stf]) rslt = setFi;
    if (str.substring(3,6) == nms[e_Kpv]) rslt = int(ctrl.Kpv * 100000);
    if (str.substring(3,6) == nms[e_Kdv]) rslt = int(ctrl.Kdv * 100000);
    if (str.substring(3,6) == nms[e_Tmi]) rslt = int(ctrl.Tmi * 1000);
    if (str.substring(3,6) == nms[e_Tiv]) rslt = int(ctrl.Tiv * 1000);
    if (str.substring(3,6) == nms[e_Tfv]) rslt = int(ctrl.Tfv * 1000);
    if (str.substring(3,6) == nms[e_Kpf]) rslt = int(ctrl.Kpf * 1000);
    if (str.substring(3,6) == nms[e_Kdf]) rslt = int(ctrl.Kdf * 1000);
    if (str.substring(3,6) == nms[e_Tif]) rslt = int(ctrl.Tif * 1000);
    if (str.substring(3,6) == nms[e_Tff]) rslt = int(ctrl.Tff * 1000);
    if (str.substring(3,6) == nms[e_Tfr]) rslt = int(ctrl.Tfm * 1000);
    if (str.substring(3,6) == nms[e_Tfm]) rslt = int(mdl.Tfm * 1000);
    if (str.substring(3,6) == nms[e_Tvr]) rslt = int(ctrl.Tvm * 1000);
    if (str.substring(3,6) == nms[e_Tvm]) rslt = int(mdl.Tvm * 1000);
    if (str.substring(3,6) == nms[e_Tor]) rslt = int(ctrl.Tom * 1000);
    if (str.substring(3,6) == nms[e_Tom]) rslt = int(mdl.Tom * 1000);
    if (str.substring(3,6) == nms[e_Ctp]) rslt = ctrl.CtrlType;
    if (str.substring(3,6) == nms[e_Klm]) rslt = int(ctrl.Klm * 1000);
    if (str.substring(3,6) == nms[e_stU]) rslt = Upwr;
    if (str.substring(3,6) == nms[e_sUm]) rslt = int(ctrl.uM);

    Serial.println(rslt);
  }
}

void ComplementaryFilter(float *output, float lfInput, float hfInput, float sampleTime)
{
  float lfWeight = 0.95;
  float hfWeight = 1.0 - lfWeight;
  float lfInt = lfInput * sampleTime;
  *output = (lfWeight * (*output + lfInt)) + (hfWeight * hfInput);
}

void doParser(){
  if (stringComplete) {
    //Serial.println(inputString);
    parse(inputString);
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
  
  if(gLog == 1){ Serial.println(millis());  }
  if(gLog == 2){ 
    Serial.print(millis()); Serial.print("\t"); 
    Serial.print(int(ctrl.uM*10)); Serial.print("\t");
    Serial.print(int(Acc_filter)); Serial.print("\t");
    Serial.println(gx/10);
  }
  if(gLog == 3){ 
    Serial.print(millis()); 
  }
  if(gLog == 4){ 
    Serial.print(millis()); 
  }
  if(gLog == 5){ 
    Serial.print(millis()); 
  }
}

uint32_t oldT = millis();
void loop() {
  if (!calibrated) {  // Калибруем только если еще не делали это
    calibration();
    calibrated = true;
    Serial.println("Calibration done! Now streaming data...");
  }
  // Читаем данные с датчика
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  uint32_t currT = millis();
  int32_t dt = currT - oldT;
  if(dt > 0){
    float fDt = 0.001 * dt;
    oldT = currT;
    ComplementaryFilter(&Acc_filter, 1.0 * gx, 1.0 * ay, fDt);
    float uM = ctrl.updateCtrl(fDt, setFi, Acc_filter, gx);
    //mdl.updateMdl(fDt, uM, ctrl.GetUi());
  }
  float u1,u2; u1 = u2 = Upwr;
  u1 -= ctrl.uM;
  u2 += ctrl.uM;
  saturate(u1, 1000, 1400);
  saturate(u2, 1000, 1400);
  if (ctrl.CtrlType){
    Motor1.writeMicroseconds(u1);
    Motor2.writeMicroseconds(u2);
  }
  else { // manual
    if(manualCmd) {
      Motor1.writeMicroseconds(u1);
      Motor2.writeMicroseconds(u2);
      manualCmd = false;
    }
    if(sU1){ Motor1.writeMicroseconds(sU1); sU1=0; }
    if(sU2){ Motor2.writeMicroseconds(sU2); sU2=0; }
  }
  doParser();
}
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    if (inChar == '\r') {
      stringComplete = true;
    }
    else inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
  }
}

void calibration() {
  long offsets[6] = {0};
  long offsetsOld[6] = {0};
  short mpuGet[6];

  // Устанавливаем стандартные диапазоны
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  // Обнуляем оффсеты
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  for (byte n = 0; n < 10; n++) {  // 10 итераций калибровки
    for (byte j = 0; j < 6; j++) {
      offsets[j] = 0;
    }

    for (byte i = 0; i < 100 + BUFFER_SIZE; i++) {
      mpu.getMotion6(&mpuGet[0], &mpuGet[1], &mpuGet[2], &mpuGet[3], &mpuGet[4], &mpuGet[5]);

      if (i >= 99) {  // Пропускаем первые 99 измерений
        for (byte j = 0; j < 6; j++) {
          offsets[j] += (long)mpuGet[j];
        }
      }
    }

    for (byte i = 0; i < 6; i++) {
      offsets[i] = offsetsOld[i] - ((long)offsets[i] / BUFFER_SIZE);
      if (i == 2) offsets[i] += 16384;  // Коррекция для оси Z
      offsetsOld[i] = offsets[i];
    }

    // Устанавливаем новые оффсеты
    mpu.setXAccelOffset(offsets[0] / 8);
    mpu.setYAccelOffset(offsets[1] / 8);
    mpu.setZAccelOffset(offsets[2] / 8);
    mpu.setXGyroOffset(offsets[3] / 4);
    mpu.setYGyroOffset(offsets[4] / 4);
    mpu.setZGyroOffset(offsets[5] / 4);
  }

  // Пересчитываем оффсеты для сохранения в EEPROM
  for (byte i = 0; i < 6; i++) {
    if (i < 3) offsets[i] /= 8;
    else offsets[i] /= 4;
  }

  // Запись оффсетов в EEPROM
  EEPROM.put(START_BYTE, offsets);
}
