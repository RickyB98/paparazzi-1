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
 * @file "modules/competition/competition_main.h"
 * @author Group 3
 * Competition Module Group 3
 */

#ifndef COMPETITION_MAIN_H
#define COMPETITION_MAIN_H

extern void competition_init();
extern void competition_loop();
extern void competition_event();
// extern void competition_datalink();

void setHeadingRate(float headingRate);
void setSpeed(float speed);

struct image_t * pic_broadcast_func(struct image_t *img);

#endif

