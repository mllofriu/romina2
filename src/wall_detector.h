/*
 * WallDetector.h
 *
 *  Created on: Jan 15, 2015
 *      Author: biorob
 */

#ifndef WALLDETECTOR_H_
#define WALLDETECTOR_H_

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include "coord_transformer.h"

using namespace ros;
using namespace boost;

class WallDetector {
public:
	WallDetector();
	virtual ~WallDetector();

private:
	void imageCallback(const sensor_msgs::Image::ConstPtr&);
	void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr);
	shared_ptr<CoordTransformer> coordTransformer;
	Subscriber imgSub;
	Subscriber infoSub;
	NodeHandle n;
};

#endif /* WALLDETECTOR_H_ */
