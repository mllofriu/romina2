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

WallDetector::WallDetector(NodeHandle n) {
	this->n = n;

	markerPub = n.advertise<Marker>("lines", 1, false);
	thrsImgPub = n.advertise<Image>("/wall_detector_image", 1, false);
	linesPub = n.advertise<romina2::PolygonsStamped>("/lines", 1, false);

	markerId = 0;

	infoSub = n.subscribe("camera_info", 2, &WallDetector::camInfoCallback,
			this);

	n.param("imgThrs", imgThrs, 150);
	n.param("lineVoteThrs", lineVoteThrs, 75);
	n.param("lineMinLen", lineMinLen, 25);
	n.param("lineMaxGap", lineMaxGap, 30);
}

WallDetector::~WallDetector() {
}

void WallDetector::imageCallback(const Image::ConstPtr& image_message) {
	ROS_DEBUG("Image received");
	CvImageConstPtr cv_ptr;
	try {
		cv_ptr = toCvShare(image_message);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Choose bottom half
	Mat roi = cv_ptr->image(Rect(0,cv_ptr->image.rows / 2,cv_ptr->image.cols, cv_ptr->image.rows/2));

	// Get Y channel
//	Mat channels[3];
//	split(cv_ptr->image, &channels[0]);
//	Mat y = channels[0];

	// Equalize Histogram
//  Mat yEq(y.size(), y.type());
//  equalizeHist( y, yEq );

	// Threshold
	Mat thrs(roi.size(), roi.type());
	threshold(roi, thrs, imgThrs, 255, THRESH_BINARY);
	//inRange(cv_ptr->image,Scalar(255 - imgThrs, 128 - imgThrs, 128 - imgThrs), Scalar(255, 128 + imgThrs, 128 + imgThrs), bin);

	Mat dilated(thrs.size(), thrs.type());
	dilate(thrs, dilated, Mat());
	/// Apply the erosion operation
	Mat eroded(dilated.size(), dilated.type());
	erode(dilated, eroded, Mat(), cv::Point(-1, -1));

	Mat origSize(cv_ptr->image.size(), cv_ptr->image.type());
	origSize = Scalar(0);
	eroded.copyTo(origSize(Rect(0,cv_ptr->image.rows / 2,cv_ptr->image.cols, cv_ptr->image.rows/2)));

	cv_bridge::CvImagePtr thrsImg(new cv_bridge::CvImage);
	thrsImg->encoding = "mono8";
	thrsImg->image = origSize;
	thrsImgPub.publish(thrsImg->toImageMsg());

	//Mat cann(thrs.size(), thrs.type());
	//Canny(thrs, cann, 0, 200);

	vector<Vec4i> lines;
	HoughLinesP(origSize, lines, 1, CV_PI / 180, lineVoteThrs, lineMinLen,
			lineMaxGap);

	//    Mat img(cv_ptr->image);
	//    for( size_t i = 0; i < lines.size(); i++ )
	//	{
	//		line( img, cv::Point(lines[i][0], lines[i][1]),
	//			cv::Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
	//	}

	//	imshow("Lines", img);
	//	imshow("Canny", cann);
	//	imshow("Thrs", thrs);
	//	imshow("Luma", y);

	vector<Polygon> linesTransformed;
	coordTransformer->transformLines(lines, image_message->header.stamp,
			linesTransformed);

	// Publish lines
	romina2::PolygonsStamped pols;
	pols.header.stamp = image_message->header.stamp;
	pols.header.frame_id = "map";
	pols.polygons = linesTransformed;
	linesPub.publish(pols);

	// Publish visual lines
	Marker m;
	m.header.frame_id = "robot";
	m.header.stamp = image_message->header.stamp;
	m.ns = "";
	m.id = markerId++;
	m.type = Marker::LINE_LIST;
	m.action = Marker::ADD;
	m.pose.orientation.w = 1.0;
	m.scale.x = .05;
	m.scale.y = .05;
	m.scale.z = .05;
	m.color.r = 1.0;
	m.color.g = 1.0;
	m.color.b = 1.0;
	m.color.a = 1.0;
	m.lifetime = Duration(1.0);
	for (int i = 0; i < linesTransformed.size(); i++) {
		vector<geometry_msgs::Point32> ps = linesTransformed.at(i).points;
		for (int j = 0; j < ps.size(); j++) {
			Point32 p32 = ps.at(j);
			geometry_msgs::Point p64;
			p64.x = p32.x;
			p64.y = p32.y;
			p64.z = p32.z;
			m.points.push_back(p64);
		}
	}
	markerPub.publish(m);

	//waitKey(3);
}

void WallDetector::camInfoCallback(
		const sensor_msgs::CameraInfo::ConstPtr info) {
	ROS_INFO("Info received");
	infoSub.shutdown();
	TransformListener * tfl = new TransformListener();
	// Initialize coordinate transformer. info->P[0] and P[5] contain fx and fy
	coordTransformer = boost::shared_ptr<CoordTransformer>(
			new CoordTransformer(info->P[0], info->P[5], info->height,
					info->width, "usb_cam", "robot", tfl));
	imgSub = n.subscribe("image", 2, &WallDetector::imageCallback, this);
}

int main(int argc, char ** argv) {
	init(argc, argv, "wall_detector");
	WallDetector w = WallDetector(NodeHandle());
	spin();
}

