/*

    Robot Class for the LBR iiwa 7 R800

    Copyright 2019-2020 Università della Campania Luigi Vanvitelli

    Author: Marco Costanzo <marco.costanzo@unicampania.it>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "sun_robot_lib/Robots/Panda.h"

using namespace TooN;
using namespace std;

namespace sun
{
/*=========CONSTRUCTORS=========*/

/*
    Full constructor
*/
Panda::Panda(const Matrix<4, 4>& n_T_e, double dls_joint_speed_saturation, const string& name)
  : Robot(transl(0.0, 0.0, 0.340), n_T_e, dls_joint_speed_saturation, name)
{
  _model = PANDA_MODEL_STR;
  // L1
  push_back_link(RobotLinkRevolute(
      // a,   alpha,     d,
      0.0, -M_PI / 2.0, 0.333,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -2.8973, 2.8973,
      // hard_velocity_limit
      2.1750,
      // string name
      "A1"));

  // L2
  push_back_link(RobotLinkRevolute(
      // a,alpha,d,
      0.0, M_PI / 2.0, 0.0,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -1.7628, 1.7628,
      // hard_velocity_limit
      2.1750,
      // string name
      "A2"));
  // L3
  push_back_link(RobotLinkRevolute(
      // a,alpha,d,
      0.0825, M_PI / 2.0, 0.3160,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -2.8973, 2.8973,
      // hard_velocity_limit
      2.1750,
      // string name
      "A3"));
  // L4
  push_back_link(RobotLinkRevolute(
      // a,alpha,d,
      -0.0825, -M_PI / 2.0, 0.0,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -3.0718, -0.0698,
      // hard_velocity_limit
      2.1750,
      // string name
      "A4"));
  // L5
  push_back_link(RobotLinkRevolute(
      // a,alpha,d,
      0.0, +M_PI / 2.0, 0.3840,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -2.8973, 2.8973,
      // hard_velocity_limit
      2.6100,
      // string name
      "A5"));
  // L6
  push_back_link(RobotLinkRevolute(
      // a,alpha,d,
      0.0880, M_PI / 2.0, 0.0,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -0.0175, 3.7525,
      // hard_velocity_limit
      2.6100,
      // string name
      "A6"));
  // L7
  push_back_link(RobotLinkRevolute(
      // a,alpha,d,
      0.0, 0.0, 0.1070,
      // robot2dh_offset, bool robot2dh_flip
      0.0, false,
      // Joint_Hard_limit_lower, Joint_Hard_limit_higher
      -2.8973, 2.8973,
      // hard_velocity_limit
      2.6100,
      // string name
      "A7"));
}

/*
    Constructor with name only
*/
Panda::Panda(const std::string& name) : Panda(Identity, 2.0, name)
{
}

/*
    Empty constructor
*/
Panda::Panda() : Panda("PANDA_NO_NAME")
{
}

}  // namespace sun

/*=========END CONSTRUCTORS=========*/