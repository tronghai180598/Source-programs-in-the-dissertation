#include "kren_model.hpp"
//#include <string>
#include <math.h>

void integr(float dt, float inp, float tm, float& out){  if (tm > 0) out += inp * dt / tm; };

void intert(float dt, float inp, float k, float tm, float& out){  if(tm>0) out += (inp * k - out) * dt / tm; };

void rldiff(float dt, float dinp, float tdf, float tm, float& out){ if(tm>0) out += dinp * tdf/tm - out * dt / tm; };

void saturate(float& val, float min, float max){  val = (val < min)?min:(val>max?max:val); }

float my_rand(float min, float max){ return 0; }

KrenMdl::KrenMdl(float Tf, float Tv, float Tm) { Tfm = Tf; Tvm = Tv; Tom = Tm; mFi = 0; mVi = 0; mAc = 0; };
KrenMdl::KrenMdl() {  Tfm = 2.0; Tvm = 0.04; Tom = 0.1; mFi = 0; mVi = 0; mAc = 0; };

void KrenMdl::setParam (float Tf, float Tv, float Tm) { Tfm = Tf; Tvm = Tv; Tom = Tm;}


/* with     SinNoise
    ________   |     ______     ______
_u_|___1____|__+_dv_|__1___|_V_|__1___|_Fi_
   |(1+sTom)|       |(s Tv)|   |(s Tf)|
   |________|       |______|   |______|
*/

float KrenMdl::updateMdl(float dt, float inp, float ul) {
  nFi = 1000;  nVi = 500;
  nFi = 0.1;  nVi = 0;
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

float KrenMdl::getFi() {return (nFi==0)?mFi:(mFi + my_rand(-nFi, nFi));}
float KrenMdl::getVi() {return (nVi==0)?mVi:(mVi + my_rand(-nVi, nVi));}
float KrenMdl::getTr() {return mAc;}

