/*
 * CoordTransformer.h
 *
 *  Created on: Jan 15, 2015
 *      Author: biorob
 */

#ifndef COORDTRANSFORMER_H_
#define COORDTRANSFORMER_H_

#include <string>
#include <tf/transform_listener.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

using namespace std;
using namespace tf;
using namespace cv;
using namespace ros;
using namespace geometry_msgs;
using namespace sensor_msgs;

class CoordTransformer {
public:
	CoordTransformer(float fx, float fy, float imgH, float imgW,
			string srcFrame, string dstFrame, TransformListener * tfl);
	void transformLines(vector<Vec4i> & lines, ros::Time stamp,
			vector<PolygonStamped> & transformedLines);
	virtual ~CoordTransformer();

private:
	Point32 intersectLine(Point32 & p, tf::Transform & t, float knownZ);

	float fx, fy, imgH, imgW;
	string srcFrame, dstFrame;
	TransformListener * tfl;
};

#endif /* COORDTRANSFORMER_H_ */
