#include "nubot_control/bezier.h"
using namespace  nubot;

Bezier::Bezier(double s, boost::ptr_list<DPoint> &controlpoint, double &p , double &q, double &diffp, double &diffq, double &double_diffp, double &double_diffq)
{
      bezier_updateflag = true;

      int n = controlpoint.size()-1;
      boost::ptr_list<DPoint>::iterator iterIntList;

      DPoint point,point1,point2;

      iterIntList = controlpoint.begin();
      p = 0;
      q = 0;
      for(int i = 0; i <= n; i ++)
      {
           point = *iterIntList;

           if(iterIntList!=controlpoint.end())
               iterIntList++;

           p = p + point.x_*Bernstein(s,i,n);
           q = q + point.y_*Bernstein(s,i,n);
      }
      diffq = diffp = 0;
      iterIntList = controlpoint.begin();

      for(int i=0; i <= n-1; i++)
      {
          point = *iterIntList;

          if(iterIntList!=controlpoint.end())
              iterIntList++;

          point1 = *iterIntList;
          diffp = diffp + (point1.x_ - point.x_)*Bernstein(s,i,n-1);
          diffq = diffq + (point1.y_ - point.y_)*Bernstein(s,i,n-1);
      }
      diffp  =  diffp*n;
      diffq  =  diffq*n;

      double_diffp = double_diffq = 0;
      iterIntList = controlpoint.begin();
      for(int i=0 ;  i  <=  n-2 ;  i ++)
      {
          point  =  *iterIntList;
          if(iterIntList!=controlpoint.end())
              iterIntList++;
          point1 =  *iterIntList;
          if(iterIntList!=controlpoint.end())
              iterIntList++;
          point2 =  *iterIntList;

          double_diffp  =  double_diffp  +  (point2.x_ -2*point1.x_ + point.x_)*Bernstein(s,i,n-2);
          double_diffq  =  double_diffq  +  (point2.y_ -2*point1.y_ + point.y_)*Bernstein(s,i,n-2);
      }

      double_diffp  =  double_diffp*n*(n-1);
      double_diffq  =  double_diffq*n*(n-1);
}

Bezier::Bezier(double s, boost::ptr_list<DPoint> &controlpoint, double &p , double &q, double &diffp, double &diffq)
{
    bezier_updateflag = true;

    int n = controlpoint.size() -1;
    boost::ptr_list<DPoint>::iterator iterIntList;
    iterIntList = controlpoint.begin();

    DPoint point,point1;
    p = 0 ;
    q = 0 ;
    for(int i = 0; i <= n; i++)
    {
        point =  *iterIntList;
        p = p + point.x_*Bernstein(s,i,n);
        q = q + point.y_*Bernstein(s,i,n);
    }
    diffq = diffp = 0;
    iterIntList = controlpoint.begin();
    for(int i = 0; i <= n-1; i++)
    {
        point =  *iterIntList;
        if(iterIntList!=controlpoint.end())
            iterIntList++;
        point1 =  *iterIntList;
        diffp  =  diffp  +  (point1.x_ -  point.x_)*Bernstein(s,i,n-1);
        diffq  =  diffq  +  (point1.y_ -  point.y_)*Bernstein(s,i,n-1);
    }
    diffp  =  diffp*n;
    diffq  =  diffq*n;
}

double Bezier::CalculatediffS(double &s,double &rho,double &phid,double v0,double c,double alpha,double p,double q,double diffp,double diffq)
{
      double rtvl;
      double x = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation().x_;
      double y = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation().y_;
      double deltax,deltay,diffs;

      if(Active_ControlPointList.size() < 3)
      {
          printf("Bezier control point is not enough");
          return 0;
      }
      deltax  = p - x ;
      deltay  = q - y;
      phid = atan2(deltay,deltax);
      while(phid<=-SINGLEPI_CONSTANT)
          phid=phid+2*SINGLEPI_CONSTANT;
      while(phid>SINGLEPI_CONSTANT)
          phid=phid-2*SINGLEPI_CONSTANT;

      rho = sqrt(deltax*deltax + deltay*deltay);
      double eps = 5;
      alpha = 1/eps;
      c = exp(alpha*rho);

      if(diffp != 0 && diffq != 0)
          diffs =c*exp(-alpha*rho)*v0 /(sqrt(diffp*diffp+diffq*diffq));

      rtvl = diffs;
      s += diffs/30;

      return  rtvl;
}

double Bezier::CalculatediffS(double &s, double &rho,double &phid,double lamda,double v0,double p,double q,double  diffp,double diffq)
{
    double thetar = atan2(diffq,diffp);
    double rtvl;
    double x  = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation().x_;
    double y  = world_model_->RobotInfo_[world_model_->AgentID_-1].getLocation().y_;

    double phi = world_model_->RobotInfo_[world_model_->AgentID_-1].getHead().radian_;

    double deltax,deltay,diffs;
    if(Active_ControlPointList.size() < 3)
    {
        printf("Bezier control point is not enough");
        return 0;
    }
    deltax  = p - x;
    deltay  = q - y;
    phid = atan2(deltay,deltax);
    rho  = sqrt(deltax*deltax + deltay*deltay);
    if(diffp != 0 && diffq != 0)
        diffs =(2*v0*cos(phid -  phi) - lamda*rho ) /(sqrt(diffp*diffp+diffq*diffq)*cos(phid - thetar));

    s += diffs/30;
    rtvl = diffs;

    return  rtvl;
}

double Bezier::CalculatePhid(double rho, double basic_phid, double thetar, double eps)
{
    double rtvl;
    if(rho  < eps)
        rtvl = basic_phid;
    else
        rtvl  =  (basic_phid*(-2*pow(rho,3)+3*eps*pow(rho,2) )+ thetar*( -2*pow(eps-rho,3) + 3*eps*pow(eps-rho,2) ) )/pow(eps,3);
    return rtvl;
}

double Bezier::Curvature(double diffp, double diffq, double double_diffp, double double_diffq)
{
    double rtvl;
    rtvl = (double_diffq*diffp - double_diffp*diffq )/pow((diffp*diffp + diffq*diffq),3/2);
    return rtvl;
}

double Bezier::Bernstein(double s , int i , int N)
{
    double  k0 = pow((double)s,(int)i);
    double  k1  =pow((double)(1-s),(int)(N-i));
    double  rtvl;
    double  k = Factorial(N-i);
    if(k==0)
    {
        return rtvl;
    }
    rtvl = k0*k1*(Factorial(N)/(Factorial(N-i)*Factorial(i)));

    return rtvl;
}


void Bezier::FromPath2Trajectory(double &s,double &rtheta ,double vr,double wr,double diffp,double diffq,double double_diffp,double double_diffq,double vprofile ,double aprofile)
{
    double dpu = sqrt(diffp*diffp+diffq*diffq);
    double dpx = (vprofile/dpu)*diffp;
    double dpy = (vprofile/dpu)*diffq;
    double du = vprofile/dpu;
    double Ts = 0.033;
    double dptduddpuu = diffp*double_diffp + diffq*double_diffq;
    double duu = aprofile/dpu - (vprofile*vprofile*dptduddpuu)/(pow(dpu,4));
    double dppx = diffp*duu +double_diffp*pow(du,2);
    double dppy = diffq*duu +double_diffq*pow(du,2);
    rtheta = atan2(dpy,dpx);
    vr = sqrt(dpx*dpx + dpy*dpy);
    wr = (dppy*dpx -dppx*dpy) / (dpx*dpx + dpy*dpy);
    s = s + vprofile*Ts/dpu + 0.5*Ts*Ts*duu;
}
