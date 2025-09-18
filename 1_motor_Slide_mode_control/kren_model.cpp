#include "kren_model.hpp"
//#include <string>
#include <math.h>

void integr(float dt, float inp, float tm, float& out){  
  if (tm > 0) out += inp * dt / tm; else out = 0;
};
void intert(float dt, float inp, float k, float tm, float& out){  
  if(tm > dt) out += (inp * k - out) * dt / tm; else out = inp * k;
};

void rldiff(float dt, float dinp, float tdf, float tm, float& out){ if(tm>0) out += dinp * tdf/tm - out * dt / tm; };

void saturate(float& val, float min, float max){  val = (val < min)?min:(val>max?max:val); }

float my_rand(float min, float max){ return 0; }

KrenMdl::KrenMdl(float Tf, float Tv, float Tm) { Tfm = Tf; Tvm = Tv; Tom = Tm; mFi = 0; mVi = 0; mAc = 0; };
KrenMdl::KrenMdl() {  Tfm = 2.0; Tvm = 0.8; Tom = 0.05; mFi = 0; mVi = 0; mAc = 0; };

void KrenMdl::setParam (float Tf, float Tv, float Tm) { Tfm = Tf; Tvm = Tv; Tom = Tm;}


/* with     SinNoise
    ________   |     ______     ______
_u_|___1____|__+_dv_|__1___|_V_|__1___|_Fi_
   |(1+sTom)|       |(s Tv)|   |(s Tf)|
   |________|       |______|   |______|
*/

float KrenMdl::updateMdl(float dt, float inp, float ul) {
  nFi = 0;  nVi = 0;
  mTms += dt;
  intert(dt, inp, 1, Tom, mAc);
  integr(dt, mAc + 
    //((nFi==0)?0:(2000*sin(mTms*425) +  3*ul*sin(mTms*125))), Tvm, mVi);
    //((nFi==0)?0:(2000*sin(mTms*425) +  0.3*ul*sin(mTms*125))), Tvm, mVi);
    //((nFi==0)?0:(200*sin(mTms*425) +  0.3*ul*sin(mTms*125))), Tvm, mVi);
    //((nFi==0)?0:(20*sin(mTms*425) +  0.03*ul*sin(mTms*125))), Tvm, mVi);
    0, Tvm, mVi);
  saturate(mVi, -32767,32767);
  integr(dt, mVi, Tfm, mFi);
  saturate(mFi, -32767,32767);
  return mFi;
};

/* without SinNoise
    ________      ______     ______
_u_|___1____|_dv_|__1___|_V_|__1___|_Fi_
   |(1+sTom)|    |(s Tv)|   |(s Tf)|
   |________|    |______|   |______|
*/
float KrenMdl::updateMdl(float dt, float inp) {
  nFi = 0;  nVi = 0;
  mTms += dt;
  intert(dt, inp, 1, Tom, mAc);
  integr(dt, mAc, Tvm, mVi);
  saturate(mVi, -32767,32767);
  integr(dt, mVi, Tfm, mFi);
  saturate(mFi, -32767,32767);
  return mFi;
};

char* KrenMdl::print() {
  //sprintf(msg, " %f %f %f", getFi()/100, getVi()/100, mAc / 100);
  return msg;
};

float KrenMdl::getFi() {return mFi;}
float KrenMdl::getVi() {return mVi;}
float KrenMdl::getTr() {return mAc;}

