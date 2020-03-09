/*
 * Copyright (C) Group 3
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/collision_trainer/collision_trainer.cpp"
 * @author Group 3
 * Module to train bebop to avoid obstacles
 */

#include <iostream>
#include <cstdio>
#include <cstdlib>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <map>

#include <sdf/sdf.hh>

gazebo::sensors::SensorManager *mgr = gazebo::sensors::SensorManager::Instance();
gazebo::sensors::ContactSensorPtr cs;

uint32_t collisions = 0;
float prevX = 0;
float prevY = 0;

float distance = 0;

void MovePillar(const std::string &pillar);
void LoopGazebo();
void InitGazebo();

void InitGazebo()
{
}

void LoopGazebo()
{
  cs = std::static_pointer_cast<gazebo::sensors::ContactSensor>(mgr->GetSensor("contactsensor"));

  gazebo::msgs::Contacts contacts = cs->Contacts();

  gazebo::physics::WorldPtr world = gazebo::physics::get_world(cs->WorldName());
  const ignition::math::Pose3d pose = world->ModelByName("bebop")->WorldPose();

  distance += sqrt(pow((float)(pose.Pos().X()) - prevX, 2) + pow((float)(pose.Pos().Y()) - prevY, 2));

  prevX = (float)(pose.Pos().X());
  prevY = (float)(pose.Pos().Y());

  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    std::string c1 = contacts.contact(i).collision1();
    std::string c2 = contacts.contact(i).collision2();
    std::string name = c2.substr(0, c2.find("::"));
    if (name.compare("cyberzoo_model") == 0)
      continue;
    ++collisions;
    if (collisions >= 3)
    {
      MovePillar(name);
    }
  }
}

void MovePillar(const std::string &pillar)
{
  gazebo::physics::WorldPtr world = gazebo::physics::get_world(cs->WorldName());
  
  world->ModelByName(pillar)->SetWorldPose(ignition::math::Pose3d(
      (double)rand() / RAND_MAX * 6. - 3.,
      (double)rand() / RAND_MAX * 6. - 3.,
      0., 0., 0., 0.));
  std::cout << "ok" << std::endl;
}

extern "C"
{
#define MODULES_DATALINK_C
#define NAV_C

#include "modules/collision_trainer/collision_trainer.h"

#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <stdio.h>
#include "math.h"
#include <time.h>

#include "modules/computer_vision/video_capture.h"
#include "modules/computer_vision/video_thread_nps.h"
#include "modules/computer_vision/cv.h"
#include "modules/competition/competition_main.h"

#include "generated/periodic_telemetry.h"
#include "lib/encoding/jpeg.h"
#include "pprzlink/messages.h"
#include "pprzlink/intermcu_msg.h"
#include "pprzlink/dl_protocol.h"

#include "subsystems/datalink/telemetry.h"
#include "subsystems/datalink/datalink.h"

#include "generated/flight_plan.h"

#include "generated/modules.h"

#define IN_TRAINING 0
#define REPOSITION 1

  uint8_t trainingState = IN_TRAINING;

  static void send_training(struct transport_tx *trans, struct link_device *dev)
  {
    pprz_msg_send_TRAINING_STATE(trans, dev, AC_ID,
                                 &distance, &collisions, &trainingState);
  }

  void collision_trainer_init()
  {
    NedCoor_f *ned = stateGetPositionNed_f();
    prevX = ned->x;
    prevY = ned->y;

    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_TRAINING_STATE, send_training);
  }

  void guidance_loop()
  {
    switch (trainingState)
    {
    case IN_TRAINING:
    {
      NedCoor_f *ned = stateGetPositionNed_f();
      if (!InsideObstacleZone(ned->x, ned->y))
      {
        trainingState = REPOSITION;
      }
    }
    break;
    case REPOSITION:
    default:
    {
      guidance_h_set_guided_pos(0., 0.);

      NedCoor_f *ned = stateGetPositionNed_f();
      if (ned->x < .1 && ned->y < .1)
      {
        trainingState = IN_TRAINING;
        distance = 0;
        collisions = 0;
      }
    }
    break;
    }
  }

  void guidance_datalink()
  {
    float speed = DL_TRAINING_ACTION_speed(dl_buffer);
    float heading = DL_TRAINING_ACTION_heading(dl_buffer);
    fprintf(stderr, "Received training speed %f, heading %f\n", speed, heading);
    setSpeed(speed);
    setHeadingRate(heading * 3.14159 / 180.);
  }

  void gazebo_loop()
  {
    LoopGazebo();
  }
}