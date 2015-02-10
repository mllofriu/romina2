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
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

#define LINE_EXTEND_FACTOR 2.0f
#define LINE_LEN_THRS 2.0f
#define IMG_BOUNDARY_THRS 10
#define LINE_MAX_DIST 1.2f

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
			vector<Polygon> & transformedLines);
	virtual ~CoordTransformer();

private:
	Point32 intersectLine(Point32 & p, tf::Transform & t, float knownZ);
	void extendLine(Point32 p1, Point32 p2, int imgX1, int imgX2, float factor,
			int img_width, Point32 & p1Ext, Point32 & p2Ext);
	float lenght(Point32 & p1, Point32 & p2);

	float fx, fy, imgH, imgW;
	string srcFrame, dstFrame;
	TransformListener * tfl;
};

#endif /* COORDTRANSFORMER_H_ */
