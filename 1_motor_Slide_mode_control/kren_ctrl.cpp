#include "kren_ctrl.hpp"
//#include <string>
#include <math.h>
enum { eManual=0, eP_Pid, eP_PDI, eSlide, eMax};

KrenCtrl::KrenCtrl(float Tfr, float Tvr, float Tor, float Tmi){
  setCtrlParam();
}
KrenCtrl::KrenCtrl(){
    Tmi = 80/1000;
    Tiv = 500/1000;
    Tfv = 5/1000;
  //Tfm *= 1.0; Tvm *= 1.0; Tom *= 1.0; // Tfr in (0.1 .. 2.0); Tvr in (0.1 .. 2); Tor in (0.7 ... 30)
  setCtrlParam();
}
void KrenCtrl::setCtrlParam(){
  if (CtrlType == eP_PDI){
   //Tfv = 0.05;
    // float Teq = Tom + Tmi + Tfv;
    // Tiv = Teq/2;
    // Kdv = Tiv*Tvm*0.5/Teq;
    // Kpv = Kdv*Tvm/(Teq);
    // Kpf = 1.0*Tfm/(2*2*Teq);
    // Tmi = 0.1;
    // Tiv = 2;
    // Tfv = 0.05;
  }
  if (CtrlType == eSlide){
    Kdv  = 3/1000;
    Kpv  = 5/1000;
    Kpf = 500/1000;
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

float lineTrans(float x, float x1, float x2, float y1, float y2){
  return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}
void KrenCtrl::UdateKalman(float Fi, float Vi){
  mFi = getFi();
  mVi = getVi();
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
  /*  float erFi = setFi - mFi;
    float setVi = erFi * Kpf;
    //integr(dt, setVi, Tvf, setVi_M);
    //err_V = (setVi_M + setVi)- Vi;
    float err_V = setVi - mVi;
    rldiff(dt, err_V-loldVi, 1, Tfv, lTr);
    loldVi = err_V;
    uI =  err_V * Kpv + lTr * Kdv - Umi;
    saturate(uI, -200,200);
    integr(dt, uI, Tmi, Umi);
    saturate(Umi, -200,200);
    integr(dt, Umi, Tiv, Ui);
    saturate(Ui, -200,200);
    uM = Umi + Ui;*/

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
    
    float eVi = Kpf * (setFi - mFi) - mVi;
    rldiff(dt, eVi-loldVi, 1, Tfv, lTr);
    loldVi = eVi;
    uI = (((Kpv * eVi + Kdv*lTr) - Ui) > 0 )? 200:(-200);
    if((uI * oUi) < 0) { 
      oUi = uI;  uI = 0; 
    } 
    oUi = uI; // через ноль
    saturate(uI, -200,200);
    integr(dt, uI, Tmi, Ui);
    saturate(Ui, -200,200);
    integr(dt, Ui, Tiv, u_PID);
    saturate(u_PID, -200,200);
    uM = Ui + u_PID;
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
  saturate(uM, -200,200);
  updateMdl(dt, Ui);
  return uM;
}
float KrenCtrl::GetUi() { return uI; }
