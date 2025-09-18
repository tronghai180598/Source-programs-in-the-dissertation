#ifndef __KrenCtrl_HPP
#define __KrenCtrl_HPP

#include "kren_model.hpp"

class KrenCtrl : public KrenMdl {
public:
  int CtrlType = 0;
  float loldVi=0, uI=0, lTr=0, uM=0, oUi = 0, Umi = 0, Ui = 0, u_PID = 0;
  float Kpm; // Parameter for max 
  float Kpf; // Parameter P of anglular contur
  float Kdf; // Parameter D of anglular contur
  float Kpv; // Parameter P of velocity contur 
  float Kdv; // Parameter D of velocity contur   
  float Tmi; // Parameter of intergrant for saturate uI
  float Tiv; // Parameter I of velocity contur for apply uI
  float Tfv; // Parameter of demph for real diff
  float Tif; // Parameter for I of intergrant for angular contur
  float Tff; // Parameter for I of intergrant after velocity 
  float Tvr = 0.8; // Parameter for velocity
  float Tfr = 2.0; // Parameter for velocity
  float Tor = 0.05; // Parameter for obekt 

  float Klm = 0.05;
  void UdateKalman(float Fi, float Vi);
  void setCtrlParam();
  KrenCtrl(float Tfr, float Tvr, float Tm, float Tmi);
  KrenCtrl();
  float updateCtrl(float dt, float setFi, float Fi, float Vi);
  float GetUi();
};

#endif /* __KrenCtrl_HPP */