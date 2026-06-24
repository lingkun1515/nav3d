#ifndef GO2_VEL_BRIDGE__SPORT_REQUEST_HPP_
#define GO2_VEL_BRIDGE__SPORT_REQUEST_HPP_

#include "unitree_api/msg/request.hpp"

namespace go2_vel_bridge
{

constexpr int32_t kSportApiStopMove = 1003;
constexpr int32_t kSportApiMove = 1008;

void make_move_request(unitree_api::msg::Request & req, float vx, float vy, float vyaw);

void make_stop_move_request(unitree_api::msg::Request & req);

}  // namespace go2_vel_bridge

#endif  // GO2_VEL_BRIDGE__SPORT_REQUEST_HPP_
