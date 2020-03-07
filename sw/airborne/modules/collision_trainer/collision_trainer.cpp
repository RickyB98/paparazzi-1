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
  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    std::string c1 = contacts.contact(i).collision1();
    std::string c2 = contacts.contact(i).collision2();

    MovePillar(c2);
  }
}

void MovePillar(const std::string &pillar)
{
  gazebo::physics::WorldPtr world = gazebo::physics::get_world(cs->WorldName());
  std::string name = pillar.substr(0, pillar.find("::"));
  if (name.compare("cyberzoo_model") == 0)
    return;
  std::cout << "Collision with " << name << ", moving pillar...";
  world->ModelByName(name)->SetWorldPose(ignition::math::Pose3d(
      (double)rand() / RAND_MAX * 6. - 3.,
      (double)rand() / RAND_MAX * 6. - 3.,
      0., 0., 0., 0.));
  std::cout << "ok" << std::endl;
  
}

extern "C"
{
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

#include "generated/periodic_telemetry.h"
#include "lib/encoding/jpeg.h"
#include "pprzlink/messages.h"
#include "pprzlink/intermcu_msg.h"

#include "subsystems/datalink/telemetry.h"

  struct image_t *pic_broadcast_func(struct image_t *img)
  {

    // Create jpg image from raw frame
    struct image_t img_jpeg;
    image_create(&img_jpeg, img->w, img->h, IMAGE_JPEG);
    jpeg_encode_image(img, &img_jpeg, 99, true);
    FILE *fp = fopen("/tmp/pprz.jpg", "w");
    fwrite(img_jpeg.buf, sizeof(uint8_t), img_jpeg.buf_size, fp);
    fclose(fp);

    return NULL;
  }

  static void send_ned_pos_func(struct transport_tx *trans, struct link_device *dev)
  {
    struct NedCoor_f *ned = stateGetPositionNed_f();
    pprz_msg_send_NED_POS(trans, dev, AC_ID, &(ned->x), &(ned->y));
  }

  void collision_trainer_init()
  {
    cv_add_to_device(&VIDEO_CAPTURE_CAMERA, pic_broadcast_func, 5);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_NED_POS, send_ned_pos_func);
  }

  void guidance_loop()
  {
  }

  void gazebo_loop()
  {
    LoopGazebo();
  }
}