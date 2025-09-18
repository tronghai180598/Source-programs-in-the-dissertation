#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include "kren_ctrl.hpp"


int main(int argc, char* argv[]) {
    KrenMdl mdl = KrenMdl();
    KrenCtrl ctrl = KrenCtrl();
    mdl.mTv *= 5.0; mdl.mTv = 1.0/20.0;
    ctrl.Kpf *= 0.6;
    printf("Kv = %f\n", 1.0 / mdl.mTv);
    printf("Kf = %f\n", 1.0 / ctrl.mTf);
    printf("Kpq = %f\n", ctrl.Kp);
    printf("Kpf = %f\n", ctrl.Kpf);
    printf("Kv = %f\n", (2*ctrl.Kpf)/(ctrl.mTf*ctrl.Kp));
    float dt = 0.001;
    char sstr[32] = "asd.log";
    float uM=0;
    float setVi = 5000;
    
    FILE * fw = fopen(sstr, "w+");
    int ticks = int(10.0 / dt);//  ticks = int(5.0 / dt);
    for (int i = 0; i < ticks; ++i)
    {
        if(i > (ticks/2)) setVi = 0;
        //if(i>1000) setVi = -2000; if(i>2000) setVi = 0;
        uM = ctrl.updateCtrl(dt, setVi, mdl.getFi(), mdl.getVi() );
        //uM = ctrl.updateCtrl(dt, setVi, ctrl.getFi(), ctrl.getVi() );
        mdl.updateMdl(dt, uM, ctrl.GetUi());        
        fprintf( fw, "%f %f %s %f %s\n", dt*i, uM*0.1, mdl.print(), ctrl.GetUi()*0.1, ctrl.print() );
        //fprintf( fw, "%f %f %s %f\n", dt*i, uM*10.0, ctrl.print(), ctrl.GetUi()*0.1 );
        
    }
    fclose(fw);
// рисуем
   FILE *gp = popen("gnuplot -persist","w"); // gp - дескриптор канала
    if (gp == NULL)
    {
        printf("Error opening pipe to GNU plot.\n");
        return 0;
    }
    if(1){
      fprintf(gp, "\
  datafile = \"asd.log\"\n\
  set terminal png font arial 20 size 1600,800\n\
  set output \"aT1.png\"\n\
  set grid x y\n\
  plot \
  datafile using 1:4 title \"mVi\" w l lw 2 lc rgb \"#8080e0\",\
  datafile using 1:3 title \"mFi\" w l lw 2 lc rgb \"#e08080\",\
  datafile using 1:8 title \"kVi\" w l lw 2 lc rgb \"#00080\",\
  datafile using 1:7 title \"kFi\" w l lw 2 lc rgb \"#800000\",\
  datafile using 1:2 title \"Um\" w l lw 2 lc rgb \"#808080\"\n\
\
  set terminal svg size 630,300 font 'arial,12'\n\
  set output \"aT1.svg\"\n\
  set grid x y\n\
  plot \
  datafile using 1:4 title \"mVi\" w l lw 1 lc rgb \"#8080e0\",\
  datafile using 1:3 title \"mFi\" w l lw 1 lc rgb \"#e08080\",\
  datafile using 1:8 title \"kVi\" w l lw 1 lc rgb \"#00080\",\
  datafile using 1:7 title \"kFi\" w l lw 1 lc rgb \"#800000\",\
  datafile using 1:2 title \"Um\" w l lw 1 lc rgb \"#808080\"\n\
"); //   datafile using 1:5 title \"mAcc\" w l lw 2 lc rgb \"#e08080\",

/*
  set terminal png font arial 20 size 1600,800\n\
  set output \"aT1.png\"\n\
  datafile using 1:4 title \"mVi\" w l lw 2 lc rgb \"#8080e0\",\
  datafile using 1:3 title \"mFi\" w l lw 2 lc rgb \"#e08080\",\
  datafile using 1:6 title \"uI\" w l lw 2 lc rgb \"#a0a0a0\",\
  datafile using 1:8 title \"kVi\" w l lw 2 lc rgb \"#00080\",\
  datafile using 1:7 title \"kFi\" w l lw 2 lc rgb \"#800000\",\
  datafile using 1:2 title \"Um\" w l lw 2 lc rgb \"#101010\"\n\

  datafile using 1:2 title \"Um\" w l lw 2 lc rgb \"#808080\",\
  datafile using 1:7 title \"kFi\" w l lw 2 lc rgb \"#800000\",\
  datafile using 1:8 title \"kVi\" w l lw 2 lc rgb \"#00080\"\n\



  set terminal svg size 630,262 font 'arial,12'\n\
  set output \"aT1.svg\"\n\
  datafile using 1:4 title \"mVi\" w l lw 1 lc rgb \"#8080e0\",\
  datafile using 1:3 title \"mFi\" w l lw 1 lc rgb \"#e08080\",\
  datafile using 1:6 title \"uI\" w l lw 1 lc rgb \"#a0a0a0\",\
  datafile using 1:8 title \"kVi\" w l lw 1 lc rgb \"#00080\",\
  datafile using 1:7 title \"kFi\" w l lw 1 lc rgb \"#800000\",\
  datafile using 1:2 title \"Um\" w l lw 1 lc rgb \"#101010\"\n\

  datafile using 1:2 title \"Um\" w l lw 1 lc rgb \"#808080\",\
  datafile using 1:7 title \"kFi\" w l lw 2 lc rgb \"#800000\",\
  datafile using 1:8 title \"kVi\" w l lw 1 lc rgb \"#00080\"\n\

*/
    }
    if(0){
      fprintf(gp, "\
  datafile = \"asd.log\"\n\
  set terminal png font arial 20 size 1024,600\n\
  set output \"aT1.png\"\n\
  set grid x y\n\
  plot \
  datafile using 1:3 title \"mFi\" w li lw 2 lc rgb \"#408080\",\
  datafile using 1:4 title \"mVi\" w li lw 2 lc rgb \"#804080\",\
  datafile using 1:7 title \"kFi\" w li lw 2 lc rgb \"#400000\",\
  datafile using 1:8 title \"kVi\" w li lw 2 lc rgb \"#400000\"\n\
");
    }
    pclose(gp);
  return 0;
}

