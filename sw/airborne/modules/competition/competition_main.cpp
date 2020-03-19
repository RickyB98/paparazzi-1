#include "competition_main.hpp"
#include <iostream>
#include <memory>

#include "modules/computer_vision/lib/vision/image.h"

#include <opencv2/core/core.hpp>
#include <opencv2/photo/photo.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cnn.hpp"

CNN* cnn;

void AiInit()
{
	cnn = new CNN();
}

uint8_t * current = nullptr;

using namespace cv;
void AiLoop()
{

	uint8_t * curr = current;
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

	Mat M(240, 520, CV_BGR2GRAY, curr);
	resize(M, M, Size(64, 64));

	cvSave("cnn.jpg", M.data);


	


}

void ParseImage(uint8_t* img)
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

	float _headingRate = 0;
	float _speed = 0;
	float _heading = 0;

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
		ParseImage((uint8_t*) (img->buf));
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
