#ifndef __KrenCtrl_HPP
#define __KrenCtrl_HPP

#include "kren_model.hpp"

class KrenCtrl : public KrenMdl {
public:
  float loldVi=0, uI=0, lTr=0, uM=0, oUi = 0;
  float Kpf, Kdf, Kp, Kd, mTmi;
  void UdateKalman(float Fi, float Vi);
  void setCtrlParam();
  KrenCtrl(float Tf, float Tv, float Tm, float Tmi);
  KrenCtrl();
  float updateCtrl(float dt, float setFi, float Fi, float Vi);
  float GetUi();
};

#endif /* __KrenCtrl_HPP */