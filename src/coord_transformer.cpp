/*
 * CoordTransformer.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: biorob
 */

#include "coord_transformer.h"

#include <sensor_msgs/PointCloud.h>
#include <tf/transform_datatypes.h>

CoordTransformer::CoordTransformer(float fx, float fy, float imgH, float imgW,
		string srcFrame, string dstFrame, TransformListener * tfl) {
	this->fx = fx;
	this->fy = fy;
	this->imgH = imgH;
	this->imgW = imgW;
	this->srcFrame = srcFrame;
	this->dstFrame = dstFrame;
	this->tfl = tfl;
}

void CoordTransformer::transformLines(vector<Vec4i> & lines, ros::Time stamp, vector<PolygonStamped> & transformedLines) {
	// Take each endpoint in the detected lines and
	// Find a point in the line of proyection to the object (3d) intersecting with the lens plane
	vector<Point32> points;
	for (int i = 0; i < lines.size(); i++) {
		Vec4i l = lines.at(i);
		Point32 p;
		p.x = -((imgW / 2) - l[0]) / fx;
		p.y = -((imgH / 2) - l[1]) / fy;
		p.z = 1;
		points.push_back(p);

		p = Point32();
		p.x = -((imgW / 2) - l[2]) / fx;
		p.y = -((imgH / 2) - l[3]) / fy;
		p.z = 1;
		points.push_back(p);
	}

	// Build point cloud with ends of each line of proyection
	PointCloud pclIn;
	pclIn.points = points;
	pclIn.header.stamp = stamp;
	pclIn.header.frame_id = srcFrame;

	// Transform the pointcloud to dstFrame
	PointCloud pclOut;
	tfl->waitForTransform(dstFrame, srcFrame, stamp, Duration(1));
	tfl->transformPointCloud(dstFrame, pclIn, pclOut);

	// Find the transform between src and dst frames
	tf::StampedTransform t;
	tfl->lookupTransform(dstFrame, srcFrame, stamp, t);

	// Proyect each line and intersect with the floor
	// Build a polygon with each proyected line
	vector<PolygonStamped> polygons;
	for(int i = 0; i < pclOut.points.size(); i += 2){
		PolygonStamped pol;
		pol.header.stamp = stamp;
		pol.header.frame_id = dstFrame;
		pol.polygon.points.push_back(intersectLine(pclOut.points.at(i), t));
		pol.polygon.points.push_back(intersectLine(pclOut.points.at(i + 1), t));

		transformedLines.push_back(pol);
	}
}

/***
 * Intersects the line formed by [t.getORigin, p] with the plane z=0
 */
Point32 CoordTransformer::intersectLine(Point32 & p, tf::Transform & t){
	Point32 pT;
	float paramLambda = t.getOrigin().getZ() / (t.getOrigin().getZ()-p.z);
	pT.x = t.getOrigin().getX() + paramLambda * (p.x - t.getOrigin().getX());
	pT.y = t.getOrigin().getY() + paramLambda * (p.y - t.getOrigin().getY());
	pT.z = 0;
	return pT;
}

CoordTransformer::~CoordTransformer() {
}

