#include <iostream>
#include <fstream>

#include <memory>

#include "modules/computer_vision/lib/vision/image.h"

//#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/photo/photo.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "cnn.hpp"

CNN* cnn;

//#define TRAINING

void AiInit()
{
	cnn = new CNN();
}

uint16_t * current = nullptr;

std::vector<float> speeds = {.5, 1., 1.5};
std::vector<float> headingRates = {-60., 0., 60.};

float _headingRate = 0;
float _speed = 0;
float _heading = 0;

void AiLoop()
{

	if (current == nullptr)
		return;

	uint8_t * curr = (uint8_t*) current;
	
	cv::Mat M(520, 240, CV_8UC2, curr); // original
	
	//cvtColor(M, M, CV_YUV2GRAY_Y422);
	//cv::imwrite("cnn.jpg", M);

	resize(M, M, cv::Size(64, 64), CV_INTER_CUBIC);
	cvtColor(M, M, CV_YUV2GRAY_UYVY);

  // convert UYVY in paparazzi to YUV in opencv
  //cvtColor(M, M, CV_YUV2RGB_Y422);
	//cvtColor(M, M, CV_RGB2GRAY);

	//cv::imwrite("cnn.png", M);
	
	cv::Mat floatArr;
	M.convertTo(floatArr, CV_32F);

	float* data = reinterpret_cast<float*>(floatArr.data);

	for (int i = 0; i < 4096; ++i) {
		data[i] /= 255.;
	}

	float* out = cnn->run(data);

	float max = out[0];
	int action = 0;
	for (int i = 1; i < 9; ++i) {
		if (out[i] > max) {
			max = out[i];
			action = i;
		}
	}
	
	int speedIdx = action % 3;
	int headingRateIdx = action / 3;

	_speed = speeds[speedIdx];
	_headingRate = headingRates[headingRateIdx];
}

void ParseImage(uint16_t * img)
{
	free(current);
	current = img;
}

extern "C"
{
#include "modules/competition/competition_main.h"

#include "modules/orange_avoider/orange_avoider_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include "modules/computer_vision/cv.h"
#include <stdio.h>
#include "math.h"
#include <time.h>

#define ORANGE_AVOIDER_VERBOSE TRUE


#define HEADING_M 0
#define HEADING_RATE_M 1

	int mode = HEADING_M;

	void setHeadingRate(float headingRate)
	{
		_headingRate = headingRate;
		mode = HEADING_RATE_M;
	}

	void setHeading(float heading)
	{
		_heading = heading;
		mode = HEADING_M;
	}

	void setSpeed(float speed)
	{
		_speed = speed;
	}

	void competition_init()
	{
		#ifndef TRAINING
		cv_add_to_device(&VIDEO_CAPTURE_CAMERA, parse_image, 10);
		#endif
	}

	void competition_loop()
	{
		switch (mode)
		{
		case HEADING_M:
		{
			guidance_h_set_guided_heading(_heading);
		}
		break;
		case HEADING_RATE_M:
		default:
		{
			guidance_h_set_guided_heading_rate(_headingRate);
		}
		break;
		}

		struct FloatEulers *eulers = stateGetNedToBodyEulers_f();

		guidance_h_set_guided_vel(_speed * cos(eulers->psi), _speed * sin(eulers->psi));
	}

	void competition_event()
	{
	}

	struct image_t * parse_image(struct image_t* img)
	{
		#ifndef TRAINING
		uint16_t * copy = (uint16_t*) malloc(img->buf_size);
		memcpy(copy, (uint16_t*) (img->buf), img->buf_size);
		ParseImage(copy);
		#endif
		return NULL;
	}

	void ai_init()
	{
		#ifndef TRAINING
		AiInit();
		#endif
	}

	void ai_loop()
	{
		#ifndef TRAINING
		AiLoop();
		#endif
	}
}