/*
 * WallDetector.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: biorob
 */

#include "wall_detector.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "romina2/PolygonsStamped.h"

using namespace visualization_msgs;
using namespace cv_bridge;
using namespace sensor_msgs;
using namespace cv;

Publisher thrsImgPub;
int imgThrs;

void imageCallback(const Image::ConstPtr& image_message) {
	ROS_DEBUG("Image received");
	CvImageConstPtr cv_ptr;
	try {
		cv_ptr = toCvShare(image_message);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Threshold
	Mat thrs(cv_ptr->image.size(), cv_ptr->image.type());
	threshold(cv_ptr->image, thrs, imgThrs, 255, THRESH_BINARY);
	//inRange(cv_ptr->image,Scalar(255 - imgThrs, 128 - imgThrs, 128 - imgThrs), Scalar(255, 128 + imgThrs, 128 + imgThrs), bin);

	cv_bridge::CvImagePtr thrsImg(new cv_bridge::CvImage);
	thrsImg->encoding = "mono8";
	thrsImg->image = thrs;
	thrsImgPub.publish(thrsImg->toImageMsg());

}

int main(int argc, char ** argv) {
	init(argc, argv, "img_thrs");
	NodeHandle n;
	n.param("imgThrs", imgThrs, 150);
	thrsImgPub = n.advertise<Image>("/thrs_img", 1, false);
	Subscriber imageSub = n.subscribe("image", 2, imageCallback);

	spin();
}

