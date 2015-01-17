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
	WallDetector(NodeHandle n);
	virtual ~WallDetector();

private:
	void imageCallback(const sensor_msgs::Image::ConstPtr&);
	void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr);
	boost::shared_ptr<CoordTransformer> coordTransformer;
	Subscriber imgSub;
	Subscriber infoSub;
	Publisher markerPub;
  Publisher thrsImgPub;
	NodeHandle n;
	int markerId;
  int imgThrs;
  int lineVoteThrs;
  int lineMinLen;
  int lineMaxGap;
};

#endif /* WALLDETECTOR_H_ */
