#ifndef __KrenMdl_HPP
#define __KrenMdl_HPP

void integr(float dt, float inp, float tm, float& out);
void intert(float dt, float inp, float k, float tm, float& out);
void rldiff(float dt, float dinp, float tdf, float tm, float& out);
void saturate(float& val, float min, float max);
float my_rand(float min, float max);


class KrenMdl {
  float mTms = 0;
  char msg[32];
  float nFi=0;
  float nVi=0;
protected:
public:
  float Tfm=0;  /// Время интергирования от скорости до значения угла гиросокпа, и определяется его параметрами.
  float Tvm=0;  /// Вhемя интергирования от приложеной силы на скорость, зависит от массы коптера и мощности двигателя
  float Tom=0;  /// время инреции частотного преобразователя двигателя 
  float mFi=0;  /// значение угла гиросокпа
  float mVi=0;  /// скорость изменения угла гиросокпа
  float mAc=0;  /// ускорение, которое пропорционально силе тяги

  KrenMdl(float Tf, float Tv, float Tm);
  KrenMdl();
  
  void setParam (float Tf, float Tv, float Tm);
  float getFi();
  float getVi();
  float getTr();
  float updateMdl(float inp, float dt, float ul);
  float updateMdl(float inp, float dt);
  char* print();
};

#endif /* __KrenMdl_HPP */