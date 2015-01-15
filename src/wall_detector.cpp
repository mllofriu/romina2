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

using namespace visualization_msgs;
using namespace cv_bridge;
using namespace sensor_msgs;
using namespace cv;

WallDetector::WallDetector() {
	markerPub = n.advertise<Marker>("lines", 1, false);

	infoSub = n.subscribe("camera_info", 2, &WallDetector::camInfoCallback,
			this);
}

WallDetector::~WallDetector() {
}

void WallDetector::imageCallback(const Image::ConstPtr& image_message) {
	ROS_INFO("Image received");
	CvImageConstPtr cv_ptr;
	try {
		cv_ptr = toCvShare(image_message, image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	Mat channels[3];
	split(cv_ptr->image, &channels[0]);
	Mat y = channels[0];

	Mat thrs(y.size(), y.type());
	threshold(y, thrs, 100, 255, THRESH_BINARY);

	Mat cann(thrs.size(), thrs.type());
	Canny(thrs, cann, 0, 200);

	vector<Vec4i> lines;
	HoughLinesP(cann, lines, 1, CV_PI / 180, 75, 50, 30);

//    Mat img(cv_ptr->image);
//    for( size_t i = 0; i < lines.size(); i++ )
//	{
//		line( img, cv::Point(lines[i][0], lines[i][1]),
//			cv::Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
//	}
//
//	imshow("Lines", img);
//	imshow("Canny", cann);
//	imshow("Thrs", thrs);
//	imshow("Luma", y);

	vector<PolygonStamped> linesTransformed;
	coordTransformer->transformLines(lines, image_message->header.stamp, linesTransformed);

	// Publish visual lines
	Marker m;
	m.header.frame_id = "robot";
	m.header.stamp = image_message->header.stamp;
	m.ns = "";
	m.id = markerId++;
	m.type = Marker::LINE_STRIP;
	m.action = Marker::ADD;;
	m.pose.orientation.w = 1.0;
	m.scale.x = .05;
	m.scale.y = .05;
	m.scale.z = .05;
	m.color.r = 1.0;
	m.color.g = 1.0;
	m.color.b = 1.0;
	m.color.a = 1.0;
	m.lifetime = Duration(1.0);
	for (int i = 0; i < linesTransformed.size(); i++){
		vector<geometry_msgs::Point32> ps = linesTransformed.at(i).polygon.points;
		for (int j = 0; j < ps.size(); j++){
			Point32 p32 = ps.at(j);
			geometry_msgs::Point p64;
			p64.x = p32.x;
			p64.y = p32.y;
			p64.z = p32.z;
			m.points.push_back(p64);
		}
	}
	markerPub.publish(m);

	waitKey(3);
}

void WallDetector::camInfoCallback(
		const sensor_msgs::CameraInfo::ConstPtr info) {
	ROS_INFO("Info received");
	infoSub.shutdown();
	TransformListener * tfl = new TransformListener();
	// Initialize coordinate transformer. info->P[0] and P[5] contain fx and fy
	coordTransformer = shared_ptr<CoordTransformer>(
			new CoordTransformer(info->P[0], info->P[5], info->height,
					info->width, "usb_cam", "robot", tfl));
	imgSub = n.subscribe("image", 2, &WallDetector::imageCallback, this);
}

int main(int argc, char ** argv) {
	init(argc, argv, "wall_detector");
	WallDetector w = WallDetector();
	spin();
}

