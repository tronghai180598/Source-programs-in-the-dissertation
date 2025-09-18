#include "kren_ctrl.hpp"
#include <string>
#include <math.h>

#define _slCtrl_2PD 1
#define _slCtrl_Modal 2
#define _slCtrl_Slide 3
#define __SelCtrl _slCtrl_Slide

KrenCtrl::KrenCtrl(float Tf, float Tv, float Tm, float Tmi){
  mTmi = Tmi;
  setParam(Tf, Tv, Tm);
  setCtrlParam();
}

KrenCtrl::KrenCtrl(){
  mTmi = 0.1;
  mTf *= 1.0; mTv *= 1.0; mTm *= 1.0; // mTf in (0.1 .. 2.0); mTv in (0.1 .. 2); mTm in (0.7 ... 30)
  setCtrlParam();
}

void KrenCtrl::setCtrlParam(){
#if (__SelCtrl==_slCtrl_2PD)
  Kd  = mTmi / ( mTm * 2 );
  Kp  = mTv * Kd / ( 2.0 * mTm * 2);
  Kpf = mTf / ( 3.125 * 2 * mTm );
  Kdf = 1.25 * mTm * Kpf / mTf;
#endif
#if (__SelCtrl==_slCtrl_Modal)
  Kd  = mTmi / ( mTm * 2 );
  Kp  = mTv / ( 2.23 * 2 * mTm);
  Kpf = mTf / ( 2.23 * 4 * mTm );
  Kdf = 1.25 * mTm * Kpf / mTf;
#endif
#if (__SelCtrl==_slCtrl_Slide)
  Kd  = 1000;
  Kp  = mTv/mTmi;
  Kpf = mTf / ( 4 * 4 * mTm );
#endif
}

//#define stabkalm 1
//#define stabkalm 0.05
#define stabkalm 0.01
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
  float kKFi = lineTrans(uI2, 0 , 100 , __KlFmn, __KlFmx);
  float kKVi = lineTrans(uI2, 0 , 100 , __KlVmn, __KlVmx);
  mFi += (Fi - mFi) * kKFi;
  mVi += (Vi - mVi) * kKVi;
}

float KrenCtrl::updateCtrl(float dt, float setFi, float Fi, float Vi){
  UdateKalman(Fi, Vi);
#if (__SelCtrl==_slCtrl_2PD)
/*   __       ___   __       __   __    ____
sFi_|+ |_eFi_|Kpf|_|+ |_eVi_|Kp|_|+ |__|_1__|_uM_
 Fi_|- |      ___ _|- |      __ _|- |  |sTmi|
    |__|  Vi_|Kdf| |__|  Ac_|Kd| |__|  |____|
*/
  float erFi = setFi - mFi;
  float setVi = erFi * Kpf - mVi * Kdf;
  lTr = getTr();
  uI =  (setVi - mVi) * Kp - lTr * Kd;
  saturate(uI, -1000,1000);
  integr(dt, uI, mTmi, uM);
  saturate(uM, -1000,1000);
  updateMdl(dt, uM);
#endif
#if (__SelCtrl==_slCtrl_Modal)
/*   __       ___   __       __   __         ____
sFi_|+ |_eFi_|Kpf|_|+ |_eVi_|Kv|_|+ |_|Kdv|_|_1__|_uM_
 Fi_|- |       Vi__|- |       Ac_|- |       |sTmi|
    |__|           |__|          |__|       |____|
*/
  uI = Kd * ( Kp * ( Kpf * (setFi - mFi) - mVi) - getTr() );
  saturate(uI, -1000,1000);
  integr(dt, uI, mTmi, uM);
  saturate(uM, -1000,1000);
  updateMdl(dt, uM);
#endif //(__SelCtrl==_slCtrl_Modal)
#if (__SelCtrl==_slCtrl_Slide)
/*   __       ___   __       __   __   ___   ____
sFi_|+ |_eFi_|Kpf|_|+ |_eVi_|Kp|_|s |_|Kdv|_|_1__|_uM_
 Fi_|- |       Vi__|- |       Ac_|g |       |sTmi|
    |__|           |__|          |n_|       |____|
*/
  float eVi = Kpf * (setFi - mFi) - mVi;
  //eVi = 1000 - mVi;
  uI = ( ( Kp * eVi  - getTr() ) > 0 )? Kd:(-Kd);
  if((uI * oUi) < 0) {  oUi = uI;  uI = 0; } oUi = uI;
  saturate(uI, -1000,1000);
  integr(dt, uI, mTmi, uM);
  saturate(uM, -1000,1000);
  updateMdl(dt, uM);
#endif //(__SelCtrl==_slCtrl_Slide)
  return uM;
}

float KrenCtrl::GetUi() { return uI; }
