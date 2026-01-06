#include "nubot_control/goaliestrategy.h"

nubot::ParabolaFitter3D::ParabolaFitter3D()
:n_(0),fly_flag_(0),data_pointer_(-1)
{
}
bool
nubot::ParabolaFitter3D::flyCheckAndAddData( const double _z_now, const DPoint& _pos_now, const double _time)
{  
#define FLY_HEIGHT_THRESH_LOW  30            //cm
  if( _z_now>FLY_HEIGHT_THRESH_LOW )
  {
    fly_flag_++;
    if(fly_flag_>4)
      fly_flag_ = 4;
    addData(_z_now,_pos_now,_time);

    z_old_ = _z_now;
    t_old_ = _time;
  }
  else
  {
    if(fly_flag_<3)
    {
      fly_flag_=0;
      clearDataBuffer();
    }
    else
      fly_flag_--;
  }
  return (fly_flag_ >= 3);
}

int
nubot::ParabolaFitter3D::addData(const double _add_z, const DPoint& _add_pos_now, const double _time)
{
  if(n_<MAXNUM_OF_FIT)
    n_++;
  data_pointer_++;

  if(data_pointer_==MAXNUM_OF_FIT)
    data_pointer_=0;

  x_[data_pointer_]=_add_pos_now.x_;
  y_[data_pointer_]=_add_pos_now.y_;
  z_[data_pointer_]=_add_z;
  t_[data_pointer_]=_time;

  return n_;
}

void
nubot::ParabolaFitter3D::fitting(double* _pfitting_err)
{//x=a0*t+a1, y=a2*t+a3, z=-HafeGravityt*t^2+a4*t+a5 (ie: z+HafeGravity*t^2=a4*t+a5)
  if(n_ < 3)
    return;

  sum_t_=0,sum_x_=0,sum_y_=0,sum_z_=0,sum_tt_=0,sum_tx_=0,sum_ty_=0,sum_tz_=0;
  t0_=getStartTime();//all the times will be minused by t0_, in order to increase the calculate precision
  for(int i=0; i<n_; i++)
  {
    sum_t_+=t_[i]-t0_;
    sum_x_+=x_[i];
    sum_y_+=y_[i];
    sum_z_+=z_[i]+HafeGravity*(t_[i]-t0_)*(t_[i]-t0_);
    sum_tt_+=(t_[i]-t0_)*(t_[i]-t0_);
    sum_tx_+=(t_[i]-t0_)*x_[i];
    sum_ty_+=(t_[i]-t0_)*y_[i];
    sum_tz_+=(t_[i]-t0_)*(z_[i]+HafeGravity*(t_[i]-t0_)*(t_[i]-t0_));
  }
  model_param_[1]=(sum_tt_*sum_x_/n_-sum_t_*sum_tx_/n_)/(sum_tt_-sum_t_*sum_t_/n_);
  model_param_[3]=(sum_tt_*sum_y_/n_-sum_t_*sum_ty_/n_)/(sum_tt_-sum_t_*sum_t_/n_);
  model_param_[5]=(sum_tt_*sum_z_/n_-sum_t_*sum_tz_/n_)/(sum_tt_-sum_t_*sum_t_/n_);
  model_param_[0]=(sum_tx_-sum_t_*sum_x_/n_)/(sum_tt_-sum_t_*sum_t_/n_);
  model_param_[2]=(sum_ty_-sum_t_*sum_y_/n_)/(sum_tt_-sum_t_*sum_t_/n_);
  model_param_[4]=(sum_tz_-sum_t_*sum_z_/n_)/(sum_tt_-sum_t_*sum_t_/n_);
  //add the "t0_"
  model_param_[1] -= model_param_[0]*t0_;
  model_param_[3] -= model_param_[2]*t0_;
  model_param_[5] +=  t0_*t0_*(-HafeGravity)-model_param_[4]*t0_;
  model_param_[4] -= 2*(-HafeGravity)*t0_;
  

  bounding_time_=(-sqrt(model_param_[4]*model_param_[4]+4*HafeGravity*(model_param_[5]))-model_param_[4])/(-2*HafeGravity);
  bounding_point_.x_=model_param_[0]*bounding_time_+model_param_[1];
  bounding_point_.y_=model_param_[2]*bounding_time_+model_param_[3];
  
  if(model_param_[0] == 0) model_param_[0] =1e-20;
    crossing_point_.x_ = GOAL_POS_X;
  crossing_time_=(crossing_point_.x_-model_param_[1])/model_param_[0];
  crossing_point_.y_=model_param_[2]*crossing_time_+model_param_[3];
  
  //the fitting error is commenly at 1e6 level
  if(_pfitting_err != NULL)
  {
    double err=0, temp;
    for(int i=0;i<n_;i++)
    {
      temp=x_[i]-model_param_[0]*t_[i]-model_param_[1];
      temp*=temp;
      err+=temp;
      temp=y_[i]-model_param_[2]*t_[i]-model_param_[3];
      temp*=temp;
      err+=temp;
      temp=z_[i]+HafeGravity*t_[i]*t_[i]-model_param_[0]*t_[i]-model_param_[1];
      temp*=temp;
      err+=temp;
    }
    *_pfitting_err = err/n_;
  }
}
void
nubot::ParabolaFitter3D::clearDataBuffer()
{
  n_=fly_flag_=0;data_pointer_=-1;z_old_=0;t_old_=0;
}
void
nubot::ParabolaFitter3D::saveFileTXT(const char* _pFilename)
{
  std::ofstream outfile( _pFilename );
  outfile<<"x\ty\tz\tt\n";
  for(int i=0; i<n_ ;i++)
    outfile<<x_[i]<<"\t"<<y_[i]<<"\t"<<z_[i]<<"\t"<<t_[i]<<std::endl;
  outfile<<"fittingX(i)="<<model_param_[0]<<"*t(i)+"<<model_param_[1]<<";\n";
  outfile<<"fittingY(i)="<<model_param_[2]<<"*t(i)+"<<model_param_[3]<<";\n";
  outfile<<"fittingZ(i)=-"<<HafeGravity<<"*t(i)*t(i)+"<<model_param_[4]<<"*t(i)+"<<model_param_[5]<<";\n";
  outfile<<"%bounding_time_="<<bounding_time_<<";bounding_point=("<<bounding_point_.x_<<" "<<bounding_point_.y_<<" 0"<<");\n";
  outfile<<"%t(n+1)="<<bounding_time_<<";x(n+1)="<<bounding_point_.x_<<";y(n+1)="<<bounding_point_.y_<<";z(n+1)=0"<<";\n";
  outfile.close();
}
double
nubot::ParabolaFitter3D::getStartTime()
{
  if(data_pointer_==-1)
    return 0;
  else if( n_==MAXNUM_OF_FIT && data_pointer_!=n_-1 )
    return t_[data_pointer_+1];
  else
   return t_[0];
}
double
nubot::ParabolaFitter3D::getEndTime()
{
  if(data_pointer_!=-1)
    return t_[data_pointer_];
  else
    return 0;
}

nubot::GoalieStrategy::GoalieStrategy()
{
  robot_info_.pos.x = -1050;  //850
  robot_info_.pos.y = 0;
  robot_info_.heading.theta = 0;
  robot_info_.vtrans.x = 0;
  robot_info_.vtrans.y = 0;
  ball_info_2d_ .pos_known = false;
  ball_info_3d_.pos_known_3d = false;
  ball_info_3d_.pos_known_2d = false;
  state_ = Move2Origin;
}
void
nubot::GoalieStrategy::setBallInfo3dRel(const nubot_common::BallInfo3d &_robot_info_3d_rel)
{
    //convert ball_info_3d_ to global axis
    DPoint robot_pos(robot_info_.pos.x,robot_info_.pos.y);
    DPoint robot_vel(robot_info_.vtrans.x,robot_info_.vtrans.y);
//    robot_pos += (time_now_-robot_info_.header.stamp.toSec())*DPoint(robot_info_.vtrans.x,robot_info_.vtrans.y);
    PPoint ball3d_poly = DPoint(_robot_info_3d_rel.pos.x,_robot_info_3d_rel.pos.y);
    DPoint ball3d_globle = robot_pos + DPoint(ball3d_poly.rotate(nubot::Angle(robot_info_.heading.theta)));
    PPoint ballvel_poly = DPoint(_robot_info_3d_rel.velocity.x,_robot_info_3d_rel.velocity.y);
    DPoint ballvel_globle = robot_vel + DPoint(ballvel_poly.rotate(nubot::Angle(robot_info_.heading.theta)));
    if( _robot_info_3d_rel.header.frame_id == "#1")
    {
      ball_info1_3d_ = _robot_info_3d_rel;
      ball_info1_3d_.pos.x = ball3d_globle.x_;
      ball_info1_3d_.pos.y = ball3d_globle.y_;
      ball_info1_3d_.velocity.x = ballvel_globle.x_;
      ball_info1_3d_.velocity.y = ballvel_globle.y_;
    }
    else
    {
      ball_info2_3d_ = _robot_info_3d_rel;
      ball_info2_3d_.pos.x = ball3d_globle.x_;
      ball_info2_3d_.pos.y = ball3d_globle.y_;
      ball_info2_3d_.velocity.x = ballvel_globle.x_;
      ball_info2_3d_.velocity.y = ballvel_globle.y_;
    }
}
bool
nubot::GoalieStrategy::ballTrack( const int THRESH_GROUND_VEL, const bool use_parabola_fitter_ )
{
  predicted_3d_ = false;
  predicted_2d_ = false;
  predictec_omi_= false;
  if( time_now_ >= ball_info1_3d_.header.stamp.toSec()+0.01 )//kinect1 data is not update yet
      ball_info1_3d_.pos_known_3d = ball_info1_3d_.pos_known_2d = false;
  if( time_now_ >= ball_info2_3d_.header.stamp.toSec()+0.01 )//kinect2 data is not update yet
      ball_info2_3d_.pos_known_3d = ball_info2_3d_.pos_known_2d = false;

  if(ball_info2_3d_.pos_known_3d || ball_info2_3d_.pos_known_2d)
    ball_info_3d_ = ball_info2_3d_;
  else
    ball_info_3d_ = ball_info1_3d_;
  time_now_ = ball_info_3d_.header.stamp.toSec();
  //kinect ball track
  if( ball_info_3d_.pos_known_3d || ball_info_3d_.pos_known_2d )//we assume that the ball_info_3d_ is more Accurate
  {
    if(use_parabola_fitter_)
    {
      if(parabola_fitter_.getEndTime()-parabola_fitter_.getStartTime()>3)//data time out
        parabola_fitter_.clearDataBuffer();
      if(parabola_fitter_.flyCheckAndAddData( ball_info_3d_.pos.z, DPoint(ball_info_3d_.pos.x,ball_info_3d_.pos.y), time_now_ ))
      if(parabola_fitter_.n_ > 4)
      {
        parabola_fitter_.fitting();
        double fitting_velocity = std::hypot(parabola_fitter_.model_param_[0], parabola_fitter_.model_param_[2]);
        if(fitting_velocity>THRESH_GROUND_VEL)
        {
          predicted_3d_ = true;
          bounding_pt_ = parabola_fitter_.bounding_point_;
          bounding_time_ = parabola_fitter_.bounding_time_;
          crossing_pt_ = parabola_fitter_.crossing_point_;
          crossing_time_dis_ = parabola_fitter_.crossing_time_-time_now_;
          std::cout<<"kinect parabola:"<<crossing_time_dis_<<std::endl;
        }
      }
    }
    //2d predict using 3d-ball
    if( !predicted_3d_ && ball_info_3d_.velocity_known )
    {
      double fitting_velocity = std::hypot(ball_info_3d_.velocity.x, ball_info_3d_.velocity.y);
      if(fitting_velocity>THRESH_GROUND_VEL*3)
      {
        predicted_2d_ = true;
        crossing_pt_.x_ = GOAL_POS_X;
        crossing_time_dis_ = (GOAL_POS_X-ball_info_3d_.pos.x)/ball_info_3d_.velocity.x;
        crossing_pt_.y_ = ball_info_3d_.pos.y+ball_info_3d_.velocity.y*crossing_time_dis_;
      }
    }
  }
  //omni vision ball track
  else if( ball_info_2d_.pos_known && ball_info_2d_.velocity_known )
  {    
    double ball_velocity = std::hypot(ball_info_2d_.velocity.x, ball_info_2d_.velocity.y);
    if( ball_velocity > THRESH_GROUND_VEL && ball_info_2d_.pos.x<-300 && ball_info_2d_.pos.x>-1000)//modify 800-1000
    {
      predictec_omi_ = true;
      crossing_pt_.x_ = GOAL_POS_X;
      crossing_time_dis_ = (GOAL_POS_X-ball_info_2d_.pos.x)/ball_info_2d_.velocity.x;
      crossing_pt_.y_ = ball_info_2d_.pos.y+ball_info_2d_.velocity.y*crossing_time_dis_;
    }
  }
  //clear timeout data
  if(predicted_3d_)
  if( crossing_time_dis_<0.06 )
  {
    parabola_fitter_.clearDataBuffer();
    predicted_3d_ = false;
  }
  if( predicted_2d_ )
  if( crossing_time_dis_<0.06 )
  {
    predicted_2d_ = false;
  }
  if(predictec_omi_ )
  if( crossing_time_dis_<0.06 )
  {
    predictec_omi_ = false;
  }

}

void
nubot::GoalieStrategy::strategy()
{
const bool CTRL_GOALIE_CAN_OUT = false;
const bool CTRL_GOALIE_FAST_REACT = true;//when the ball is realy near the goal in 3D, goalie can move to block
const bool CTEL_GOALIE_CAN_TURN = false;

#define thresh_lim(amin,a,amax)  (std::max(amin, std::min(amax,(a))))
#define GOALIE_ANGLE_THRESH    (M_PI/6.0) //30 degree
#define GoalieFieldX  -1070  //-750 //the thresh of goalie movement if CTRL_GOALIE_CAN_OUT, and CTRL_GOALIE_FAST_REACT if ball is in this thresh
#define GoalieFieldY  130  //(goal length is -100~100) used to check if the ball will goal

  DPoint robot_pos(robot_info_.pos.x,robot_info_.pos.y);
  double robot_angle = robot_info_.heading.theta;
  bool ball_seen_omni = ball_info_2d_.pos_known;
  DPoint ball_pos_omni = DPoint(0,0);
  if(ball_seen_omni)
    ball_pos_omni = DPoint(ball_info_2d_.pos.x,ball_info_2d_.pos.y);
  DPoint ball_pos_3d = DPoint(ball_info_3d_.pos.x,ball_info_3d_.pos.y);
  double ball_angle_omni = (PPoint(ball_pos_omni).angle_-PPoint(robot_pos).angle_).radian_;
  static double ball_angle_omni_last = 0;
  ball_angle_omni = thresh_lim(-GOALIE_ANGLE_THRESH,ball_angle_omni,GOALIE_ANGLE_THRESH);
  double ball_angle_3d = (PPoint(ball_pos_3d).angle_-PPoint(robot_pos).angle_).radian_;
  ball_angle_3d = thresh_lim(-GOALIE_ANGLE_THRESH,ball_angle_3d,GOALIE_ANGLE_THRESH);
  bool predicted = predicted_3d_||predicted_2d_||predictec_omi_;

  debug_str_ = "";

  static bool moving2ball = false;//mark if the goalie should move to block the ball

  if( predicted )
  {
    if( fabs(crossing_pt_.y_)<GoalieFieldY && crossing_time_dis_>0 && crossing_time_dis_<5 )
    {
      dest_point_ = crossing_pt_;
      moving2ball = true;
      state_ = Move2Ball;
    }
  }
  if( CTRL_GOALIE_FAST_REACT )
  {
      if( (ball_info_3d_.pos_known_3d||ball_info_3d_.pos_known_2d) && ball_info_3d_.pos.x<GoalieFieldX && fabs(ball_info_3d_.pos.y)<GoalieFieldY && ball_info_3d_.pos.z>30)//ball is near in 3D
      {
          dest_point_ = DPoint(GOAL_POS_X,int(ball_info_3d_.pos.y));
          moving2ball = true;
          state_ = Move2Ball;
      }
      else if(ball_seen_omni && ball_pos_omni.x_>-890 && ball_pos_omni.x_<GoalieFieldX-50 && fabs(ball_pos_omni.y_)<GoalieFieldY)
      {
          dest_point_ = DPoint(GOAL_POS_X,int(ball_pos_omni.y_));
          moving2ball = true;
          state_ = Move2Ball;
      }
  }
//std::cout<<"ballsee:"<<(ball_info_3d_.pos_known_3d||ball_info_3d_.pos_known_2d)
//        <<" pos:"<<ball_info_3d_.pos.x<<","<<ball_info_3d_.pos.y<<","<<ball_info_3d_.pos.z
//        <<" vel:"<<ball_info_3d_.velocity.x<<","<<ball_info_3d_.velocity.y<<","<<ball_info_3d_.velocity.z<<std::endl;
//std::cout<<"ballsee:"<<ball_info_2d_.pos_known
//        <<" pos:"<<ball_info_2d_.pos.x<<","<<ball_info_2d_.pos.y
//        <<" vel:"<<ball_info_2d_.velocity.x<<","<<ball_info_2d_.velocity.y<<std::endl;

  static int idle_cnt = 0;//use for time delay in StandBy state
  static GoalieState last_state=StandBy;
  //ball_is_near marked if the "omnivision" saw the ball near goal
  bool ball_is_near = false;
  if(CTRL_GOALIE_CAN_OUT)
    ball_is_near = ball_pos_omni.x_ < GoalieFieldX && fabs(ball_pos_omni.y_)<GoalieFieldY
            && ball_pos_omni.x_ > robot_pos.x_+20
            && ball_seen_omni
            && !moving2ball;
  //turn2ball_omni means kinect cant see ball, and robot should turns fast untill kinect can see
  bool turn2ball_omni = !(ball_info_3d_.pos_known_3d || ball_info_3d_.pos_known_2d)
                        && ball_seen_omni
                        && !ball_is_near
                        && fabs(robot_angle - ball_angle_omni) > M_PI/30;
  bool far_from_origin = robot_pos.x_ > GoalieFieldX || fabs(robot_pos.y_)>GoalieFieldY;
  
  switch(state_)
  {
  case StandBy:
    thresh_vel_ = 0;
    if( (ball_info_3d_.pos_known_3d||ball_info_3d_.pos_known_2d) && fabs(robot_angle - ball_angle_3d)>M_PI/30 && !far_from_origin )
    {
      dest_angle_ = ball_angle_3d;
      thresh_omiga_ = M_PI/3;
    }
    else if(!ball_seen_omni && !far_from_origin)
    {
      dest_angle_ = ball_angle_omni_last;
      thresh_omiga_ = M_PI/6;
    }
    else
    {
      thresh_omiga_ = 0;
    }
    //old code: idle_cnt > 15;this parameter decide the delay time of goallie for catch ball
    state_ = moving2ball                                                 ? Move2Ball  :
            ball_is_near                                                 ? Move2Ball  :     
            idle_cnt>35 || far_from_origin                               ? Move2Origin:
            turn2ball_omni && (robot_pos-DPoint(GOAL_POS_X,0)).length()<10 ? Turn2Ball  :
            state_;
    if (state_ != StandBy)
      idle_cnt = 0;
    else
      idle_cnt++;
    debug_str_ = "StandBy:angle=" + boost::lexical_cast<std::string>(dest_angle_);
    break;

  // when state_ changed to Move2Ball, the dest_point_ is already set
  case Move2Ball:
    if (ball_is_near)
    {
      dest_point_ = ball_pos_omni;
    }
    dest_point_.y_ = thresh_lim(double(-GOAL_POS_Y),dest_point_.y_,double(GOAL_POS_Y));
    dest_point_.x_ = thresh_lim(double(GOAL_POS_X),dest_point_.x_,double(GoalieFieldX));
    thresh_vel_ = 300; //300
    dest_angle_ = 0;//???????? is that right?
    thresh_omiga_ = M_PI/3;
    if( (robot_pos-dest_point_).length() < 10 )
    {
      moving2ball = false;
      state_ = StandBy;
    }
    if(ball_is_near)
      debug_str_ = "Move2Ball near:(";
    else
      debug_str_ = "Move2Ball cross:(";
    debug_str_ += boost::lexical_cast<std::string>(dest_point_.x_) + "," + boost::lexical_cast<std::string>(dest_point_.y_) + ")";
      
    break;

  case Turn2Ball:
    if ( fabs(robot_angle-ball_angle_omni)<M_PI/30 )
    {
      state_ = StandBy;
    }
    else 
    {
      if(!ball_seen_omni)
      {
        state_ = StandBy;          
      }        
      else
      {
        thresh_vel_ = 0;
        dest_angle_ = ball_angle_omni;
        thresh_omiga_ = M_PI/3;
        debug_str_ = "Turn2Ball:" + boost::lexical_cast<std::string>(dest_angle_);
      }
    }
    state_ = moving2ball  ? Move2Ball :
             ball_is_near ? Move2Ball :
             state_;
    
    break;

  case Move2Origin:
    if ( (robot_pos-DPoint(GOAL_POS_X,0)).length()<10 )
      state_ = StandBy;
    else
    {
      dest_point_ = DPoint(GOAL_POS_X,0);
      thresh_vel_ = 100;
      if(!ball_seen_omni)
        dest_angle_ = ball_angle_omni_last;
      else 
        dest_angle_ = ball_angle_omni;
      thresh_omiga_ = M_PI/3;
      debug_str_ = "Move2Origin";
    }
    state_ = moving2ball  ? Move2Ball :
            ball_is_near ?  Move2Ball:
            state_;
    break;
  default:
    state_ = StandBy;
  }
  if(!CTEL_GOALIE_CAN_TURN)
    dest_angle_ = 0;

  if(ball_seen_omni)
    ball_angle_omni_last = ball_angle_omni;
  else
  {
    if(ball_angle_omni_last>0)
      ball_angle_omni_last -= M_PI/180.0;
    else
      ball_angle_omni_last += M_PI/180.0;
  }

}

