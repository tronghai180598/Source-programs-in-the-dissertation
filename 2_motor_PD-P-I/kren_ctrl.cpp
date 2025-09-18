#include "kren_ctrl.hpp"
//#include <string>
#include <math.h>
enum { eManual=0, eP_Pid, eP_PDI, eSlide, eMax};

KrenCtrl::KrenCtrl(float Tfr, float Tvr, float Tor, float Tmi){
  setCtrlParam();
}

KrenCtrl::KrenCtrl(){
  Tmi = 0.1;
  Tfv = 0.05;
  //Tfm *= 1.0; Tvm *= 1.0; Tom *= 1.0; // Tfr in (0.1 .. 2.0); Tvr in (0.1 .. 2); Tor in (0.7 ... 30)
  setCtrlParam();
}

void KrenCtrl::setCtrlParam(){
  if (CtrlType == eP_PDI){
    //Tfv = 0.05;
    float Teq = Tom + Tmi + Tfv;
    Tiv = Teq/2;
    Kdv = Tiv*Tvm*0.5/Teq;
    Kpv = Kdv*Tvm/(Teq);
    Kpf = 1.0*Tfm/(2*2*Teq);
  /*
    Kdv  = Tmi / ( Tom * 2 );
    Kpv  = Tvm * Kdv / ( 2.0 * Tom * 2);
    Kpf = Tfm / ( 3.125 * 2 * Tom );
    Kdf = 1.25 * Tom * Kpf / Tfm;*/
  }
  if (CtrlType == eSlide){
    Kdv  = 1000;
    //Kpv  = Tvr/Tmi;
    //Kpf = Tfr / ( 4 * 4 * Tor );
  }
  if (CtrlType == eMax){
/*   __       ___   __            __   __   ____
sFi_|+ |_eFi_|Kpf|_|+ |_eVi__Kp__|s |_|Kd|_|_1__|_uM_
 Fi_|- |       Vi__|- |  |AC|*Ac_|g |      |sTmi|
    |__|           |__|          |n_|      |____|
*/
    //Kdv  = Tmi / ( Tor * 2 );
    //Kpv  = Tvr / ( 2 * Tor);
    //Kpm  = 2*Kdv/(Tvr*Tmi);
    //Kpf = Tfr / ( 2.2 * 2.5 * Tor ); // opt
    //Kpf = Tfr / ( 5 * 5 * Tor );
  }
}

#define stabkalm 1
//#define stabkalm 0.5
//#define stabkalm 0.05
//#define stabkalm 0.01
//#define __Kalman
#ifdef  __Kalman
  #define __KlFmn 0.0042
  #define __KlFmx 0.52
  #define __KlVmn 0.042
  #define __KlVmx 0.52
#else
  #define __KlFmn stabkalm
  #define __KlFmx stabkalm
  #define __KlVmn stabkalm
  #define __KlVmx stabkalm
#endif // 

float lineTrans(float x, float x1, float x2, float y1, float y2){
  return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}
void KrenCtrl::UdateKalman(float Fi, float Vi){
  float uI2 = ((uI/100)*(uI/100));
  float kKFi = Klm; // lineTrans(uI2, 0 , 100 , __KlFmn, __KlFmx);
  float kKVi = Klm; // lineTrans(uI2, 0 , 100 , __KlVmn, __KlVmx);
  mFi += (Fi - mFi) * kKFi;
  mVi += (Vi - mVi) * kKVi;
}

float KrenCtrl::updateCtrl(float dt, float setFi, float Fi, float Vi){
  UdateKalman(Fi, Vi);
  if (CtrlType == eP_PDI){
/*   __       ___   __       __   __    ____
sFi_|+ |_eFi_|Kpf|_|+ |_eVi_|Kp|_|+ |__|_1__|_uM_
 Fi_|- |      ___ _|- |      __ _|- |  |sTmi|
    |__|  Vi_|Kdf| |__| dV1_|Kd| |__|  |____|
*/
    float erFi = setFi - Fi;
    float setVi = erFi * Kpf ;
    //integr(dt, setVi, Tvf, setVi_M);
    //err_V = (setVi_M + setVi)- Vi;
    float err_V = setVi - Vi;
    rldiff(dt, err_V-loldVi, 1, Tfv, lTr);
    loldVi = err_V;
    uI =  err_V * Kpv + lTr * Kdv - Umi;
    saturate(uI, -1000,1000);
    integr(dt, uI, Tmi, Umi);
    saturate(Umi, -1000,1000);
    integr(dt, Umi, Tiv, Ui);
    saturate(Ui, -200,200);
    uM = Umi + Ui;
    saturate(uM, -200,200);


/*
    float erFi = setFi - mFi;
    float setVi = erFi * Kpf - mVi * Kdf;
    lTr = getTr();
    uI =  (setVi - mVi) * Kpv - lTr * Kdv;
    saturate(uI, -1000,1000);
    integr(dt, uI, Tmi, uM);*/
  }
  if (CtrlType == eSlide){
/*   __       ___   __       __   __   ___   ____
sFi_|+ |_eFi_|Kpf|_|+ |_eVi_|Kp|_|s |_|Kdv|_|_1__|_uM_
 Fi_|- |       Vi__|- |       Ac_|g |       |sTmi|
    |__|           |__|          |n_|       |____|
*/
    /*
    float eVi = Kpf * (setFi - mFi) - mVi;
    //eVi = 1000 - mVi;
    uI = ( ( Kpv * eVi  - getTr() ) > 0 )? Kdv:(-Kdv);
    if((uI * oUi) < 0) {  oUi = uI;  uI = 0; } oUi = uI; // через ноль
    saturate(uI, -1000,1000);
    integr(dt, uI, Tmi, uM);
    saturate(uM, -1000,1000);
    updateMdl(dt, uM);*/
  }
  if (CtrlType == eMax){
/*   __       ___   __            __   ___   ____
sFi_|+ |_eFi_|Kpf|_|+ |_eVi__Kp__|s |_|Kdv|_|_1__|_uM_
 Fi_|- |       Vi__|- |  |AC|*Ac_|g |       |sTmi|
    |__|           |__|          |n_|       |____|
*/
    /*
    float eVi = Kpf * (setFi - mFi) - mVi;

    float xp = getTr();
    if( (eVi*eVi + 1.4*xp*xp) < 2000000 ){
      uI = Kdv * ( Kpv * eVi - getTr() );
    }
    else{
      // maxFast method
      if(xp > 0) xp = getTr(); else xp = -getTr();
      uI = ( ( Kpm * eVi  + xp ) > 0 )? 1000:(-1000);
      if((uI * oUi) < 0) {  oUi = uI;  uI = 0; } oUi = uI; // через ноль
    }*/
  }
  updateMdl(dt, uM);
  return uM;
}

float KrenCtrl::GetUi() { return uI; }
