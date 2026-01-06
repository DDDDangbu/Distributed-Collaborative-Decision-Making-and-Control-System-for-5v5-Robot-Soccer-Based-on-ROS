#ifndef DRIBBLE_STATUS_HPP
#define DRIBBLE_STATUS_HPP
#include "core.hpp"
class DribbleState
{
public:
  bool is_dribble_;

  nubot::DPoint satrt_point_;

  DribbleState()
      :is_dribble_(false)
  {
  }

  void reset()
  {
      is_dribble_ = false;
  }

  void set(const nubot::DPoint &_pos_satrt)
  {
      is_dribble_ = true;
      satrt_point_ = _pos_satrt;
  }   
  /**
   * @brief                 更新抓球信息
   * @param _is_dribble     调用抓球服务后返回的response
   * @param _robot_pos      机器人在世界坐标系下的位置
   * @param _ball_robot     足球在机器人体坐标系下的位置
   */
  void update( const bool &_is_dribble, const nubot::DPoint &_robot_pos, const nubot::DPoint &_ball_robot)
  {
      if( _is_dribble == is_dribble_ )
          return;
      if( _is_dribble)
          set(_robot_pos);
      /// 是否可以取消，没有前向视觉后，机器人带上球对足球位置的判断十分不准确，所以这个判断容易出现问题
//      else if(fabs(_ball_robot.x_) > ConstDribbleX || fabs(_ball_robot.y_) > ConstDribbleY)
//          reset();
      else if(!_is_dribble)
          reset();
  }

  nubot::DPoint limitWithinCircle( const nubot::DPoint &_point_in, const int _radius = 300)
  {
      if( (_point_in-satrt_point_).length()<_radius )
		  return _point_in;
	  else
          return satrt_point_ + (_point_in-satrt_point_) *( _radius/(_point_in-satrt_point_).length() );
  }
};

#endif
