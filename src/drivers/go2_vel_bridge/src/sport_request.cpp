#include "go2_vel_bridge/sport_request.hpp"

#include <cstdio>

namespace go2_vel_bridge
{

void make_move_request(unitree_api::msg::Request & req, float vx, float vy, float vyaw)
{
  char buffer[128];
  std::snprintf(
    buffer, sizeof(buffer), "{\"x\":%.6f,\"y\":%.6f,\"z\":%.6f}", vx, vy, vyaw);
  req.header.identity.api_id = kSportApiMove;
  req.parameter = buffer;
}

void make_stop_move_request(unitree_api::msg::Request & req)
{
  req.header.identity.api_id = kSportApiStopMove;
  req.parameter.clear();
}

}  // namespace go2_vel_bridge
