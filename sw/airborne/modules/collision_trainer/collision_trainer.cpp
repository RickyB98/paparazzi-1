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

#define IN_TRAINING 0
#define REPOSITION 1

uint8_t trainingState = IN_TRAINING;

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

bool runOnce = false;

void LoopGazebo()
{

  cs = std::static_pointer_cast<gazebo::sensors::ContactSensor>(mgr->GetSensor("contactsensor"));

  gazebo::msgs::Contacts contacts = cs->Contacts();

  gazebo::physics::WorldPtr world = gazebo::physics::get_world(cs->WorldName());
  if (!runOnce) {
    runOnce = false;
    
  }

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
    if (name.compare("bebop") == 0)
      name = c1.substr(0, c1.find("::"));
      
    ++collisions;
    if (collisions >= 3 || trainingState == REPOSITION)
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


  float recSpeed = 0.;
  float recHeading = 0.;

  float min(float f1, float f2) {
    return f1 > f2 ? f2 : f1;
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
      } else {
        setSpeed(recSpeed);
        setHeadingRate(recHeading);
      }
    }
    break;
    case REPOSITION:
    default:
    {

      NedCoor_f *ned = stateGetPositionNed_f();
      float psi = atan2(ned->y, ned->x) + 3.14159;

      while (psi > 2 * 3.14159) {
        psi -= 2 * 3.14159;
      }
      while (psi < 0) {
        psi += 2 * 3.14159;
      }

      // struct FloatEulers * eulers = stateGetNedToBodyEulers_f();

      // float psi_m = eulers->psi;

      // while (psi_m > 2 * 3.14159) {
      //   psi_m -= 2 * 3.14159;
      // }
      // while (psi_m < 0) {
      //   psi_m += 2 * 3.14159;
      // }

      // float err = (psi - psi_m) * ((ned->x * ned->y) > 0 && (abs(psi - psi_m) > .35) ? 1 : -1);

      setHeading(psi);
      setSpeed(2.);

      fprintf(stderr, "psi: %f, x: %f, y: %f, \n", psi, ned->x, ned->y);

      if (abs(ned->x) < .1 && abs(ned->y) < .1)
      {
        trainingState = IN_TRAINING;
        distance = 0;
        collisions = 0;
        recSpeed = 0.;
        recHeading = 0.;
      }
    }
    break;
    }
  }

  void guidance_datalink()
  {
    recSpeed = DL_TRAINING_ACTION_speed(dl_buffer);
    recHeading = DL_TRAINING_ACTION_heading(dl_buffer);
    fprintf(stderr, "Received training speed %f, heading %f\n", recSpeed, recHeading);
  }

  void gazebo_loop()
  {
    LoopGazebo();
  }
}