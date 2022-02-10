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

#ifndef ROBOTPANDA_H
#define ROBOTPANDA_H

#include "sun_robot_lib/Robot.h"

#define PANDA_MODEL_STR "Panda"

namespace sun
{
class Panda : public Robot
{
public:
  /*!
      Full constructor
  */
  Panda(const TooN::Matrix<4, 4>& n_T_e, double dls_joint_speed_saturation, const std::string& name);

  /*!
      Constructor with name only
  */
  Panda(const std::string& name);

  /*!
      Empty constructor
  */
  Panda();
};

}  // namespace sun

#endif