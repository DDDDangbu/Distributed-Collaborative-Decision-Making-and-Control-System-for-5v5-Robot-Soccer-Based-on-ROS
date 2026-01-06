#include "nubot_control/behaviour.hpp"
using namespace nubot;

Behaviour::Behaviour()
{
    move_action_=No_Action;
    rotate_action_=No_Action;
    maxvel_       = 0 ;                       //最大速度限制
    target_ori_   = 0 ;                   //目标点角度
    maxw_         = 0;                         //最大角速度限制
    rotate_mode_  = 0;                  //旋转方式
}
Behaviour::~Behaviour()
{
}
void Behaviour::clear()
{
    move_action_=No_Action;
    rotate_action_=No_Action;
}

void
Behaviour::fuzzyPIDcontrol(float &deltakp, float &deltaki,float &deltakd, float err,float err1)
{
    int kp[7][7]={{PB,PB,PM,PM,PS,ZO,ZO},
                  {PB,PB,PM,PS,PS,ZO,ZO},
                  {PM,PM,PM,PS,ZO,NS,NS},
                  {PM,PM,PS,ZO,NS,NM,NM},
                  {PS,PS,ZO,NS,NS,NM,NM},
                  {PS,ZO,NS,NM,NM,NM,NB},
                  {ZO,ZO,NM,NM,NM,NB,NB}};

    int kd[7][7]={{PS,NS,NB,NB,NB,NM,PS},
                  {PS,NS,NB,NM,NM,NS,ZO},
                  {ZO,NS,NM,NM,NS,NS,ZO},
                  {ZO,NS,NS,NS,NS,NS,ZO},
                  {ZO,ZO,ZO,ZO,ZO,ZO,ZO},
                  {PB,NS,PS,PS,PS,PS,PB},
                  {PB,PM,PM,PM,PS,PS,PB}};

    int ki[7][7]={{NB,NB,NM,NM,NS,ZO,ZO},
                  {NB,NB,NM,NS,NS,ZO,ZO},
                  {NB,NM,NS,NS,ZO,PS,PS},
                  {NM,NM,NS,ZO,PS,PM,PM},
                  {NM,NS,ZO,PS,PS,PM,PB},
                  {ZO,ZO,PS,PS,PM,PB,PB},
                  {ZO,ZO,PS,PM,PM,PB,PB}};

    float e  = err;
    float ec = err - err1;
    float es[7],ecs[7];

    es[NB]=ufl(e,-30,-10);  //e
    es[NM]=uf(e,-30,-20,0);
    es[NS]=uf(e,-30,-10,10);
    es[ZO]=uf(e,-20,0,20);
    es[PS]=uf(e,-10,10,30);
    es[PM]=uf(e,0,20,30);
    es[PB]=ufr(e,10,30);

    ecs[NB]=ufl(ec,-3,-1);//ec
    ecs[NM]=uf(ec,-3,-2,0);
    ecs[NS]=uf(ec,-3,-1,1);
    ecs[ZO]=uf(ec,-2,0,2);
    ecs[PS]=uf(ec,-1,1,3);
    ecs[PM]=uf(ec,0,2,3);
    ecs[PB]=ufr(ec,1,3);

    float form[7][7];
    int i,j;
    float w,h,r;

    for(i=0;i<7;i++)
    {
      for(j=0;j<7;j++)
     {
        h=es[i];
        r=ecs[j];
        w=fand(h,r);
        form[i][j]=w;
     }
    }

    int a=0,b=0;
    for(i=0;i<7;i++)
    {
     for(j=0;j<7;j++)
     {
         if(form[a][b]<form[i][j])
     {
             a=i;
             b=j;
     }
     }
    }
    float lsd;
    int p,d,in;
    lsd=form[a][b];
    p=kp[a][b];
    d=kd[a][b];
    in=ki[a][b];

    float detkp,detkd,detki;
    if(p==NB)
    detkp=cufl(lsd,-0.5,-0.3);
    else if(p==NM)
    detkp=cuf(lsd,-0.5,0.3,0);
    else if(p==NS)
    detkp=cuf(lsd,-0.5,0.3,0.3);
    else if(p==ZO)
    detkp=cuf(lsd,-0.3,0,0.3);
    else if(p==PS)
    detkp=cuf(lsd,-0.3,0.3,0.5);
    else if(p==PM)
    detkp=cuf(lsd,0,0.3,0.5);
    else if(p==PB)
    detkp=cufr(lsd,0.3,0.5);


    if(d==NB)
    detkd=cufl(lsd,-3,-1);
    else if(d==NM)
    detkd=cuf(lsd,-3,2,0);
    else if(d==NS)
    detkd=cuf(lsd,-3,1,1);
    else if(d==ZO)
    detkd=cuf(lsd,-2,0,2);
    else if(d==PS)
    detkd=cuf(lsd,-1,1,3);
    else if(d==PM)
    detkd=cuf(lsd,0,2,3);
    else if(d==PB)
    detkd=cufr(lsd,1,3);

    if(in==NB)
    detki=cufl(lsd,-0.06,-0.02);
    else if(in==NM)
    detki=cuf(lsd,-0.06,-0.04,0);
    else if(in==NS)
    detki=cuf(lsd,-0.06,-0.02,0.02);
    else if(in==ZO)
    detki=cuf(lsd,-0.04,0,0.04);
    else if(in==PS)
    detki=cuf(lsd,-0.02,0.02,0.06);
    else if(in==PM)
    detki=cuf(lsd,0,0.04,0.06);
    else if (in==PB)
    detki=cufr(lsd,0.02,0.06);


    deltakp = detkp;
    deltaki = detki;
    deltakd = detkd;
}

void
Behaviour::setTarget(DPoint target, float maxvel, char move_action, DPoint target_vel)
{
    target_=target;
    maxvel_=maxvel;
    move_action_=move_action;
    target_vel_=target_vel;
}

void
Behaviour::setOrienation(float target_ori, float maxw, char rotate_action, int rotate_mode,double tar_w)
{
    target_ori_=target_ori;
    target_w_  =tar_w;
    maxw_=maxw;
    rotate_action_=rotate_action;
    rotate_mode_=rotate_mode;
}
