#include <iostream>

#include <memory>

#include "modules/computer_vision/lib/vision/image.h"

//#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/photo/photo.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "cnn.hpp"

CNN* cnn;

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

	uint16_t * curr = current;
	if (current == nullptr)
		return;

	std::vector<float> grayScale = std::vector<float>(520 * 240);

	for (int i = 0; i < 520 * 240; ++i) {
		grayScale[i] = (float) curr[i * 4] / 256;
		if (i < 10) {
			std::cout << grayScale[i] << std::endl;
		}
	}
	std::cout << "end" << std::endl;

	cv::Mat M(240, 520, CV_8UC2, curr); // original

  // convert UYVY in paparazzi to YUV in opencv
  //cvtColor(M, M, CV_YUV2RGB_Y422);
	//cvtColor(M, M, CV_RGB2GRAY);
	cvtColor(M, M, CV_YUV2GRAY_Y422);
	
	cv::Mat floatArr;
	M.convertTo(floatArr, CV_32F);

	resize(floatArr, floatArr, cv::Size(64, 64));

	cv::imwrite("cnn.jpg", floatArr);
	float* out = cnn->run(reinterpret_cast<float*>(floatArr.data));

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

void ParseImage(uint16_t* img)
{
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
		cv_add_to_device(&COMPETITION_CAMERA_FRONT, parse_image, 10);
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
		fprintf(stderr, "type: %d\n", img->type);
		ParseImage((uint16_t*) (img->buf));
	}

	void ai_init()
	{
		AiInit();
	}

	void ai_loop()
	{
		AiLoop();
	}
}