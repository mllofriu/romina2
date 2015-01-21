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

void CoordTransformer::transformLines(vector<Vec4i> & lines, ros::Time stamp,
		vector<Polygon> & transformedLines) {
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
	try {
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
		vector<Polygon> polygons;
		for (int i = 0; i < pclOut.points.size(); i += 2) {
			Polygon pol;
			Point32 p1 = intersectLine(pclOut.points.at(i), t, 0.05f);
			Point32 p2 = intersectLine(pclOut.points.at(i + 1), t, 0.05f);

			// Check that the detected points are valid
			// Camera could never detect something behind the robot
			// but the assumption of z == 0.05 could produce this
			if (p1.x > 0 && p2.x > 0) {
				pol.points.push_back(p1);
				pol.points.push_back(p2);

				transformedLines.push_back(pol);
			}
		}
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	}
}

/***
 * Intersects the line formed by [t.getORigin, p] with the plane z=0
 */
Point32 CoordTransformer::intersectLine(Point32 & p, tf::Transform & t,
		float knownZ) {
	Point32 pT;
	float paramLambda = (knownZ - t.getOrigin().getZ())
			/ (p.z - t.getOrigin().getZ());
	pT.x = t.getOrigin().getX() + paramLambda * (p.x - t.getOrigin().getX());
	pT.y = t.getOrigin().getY() + paramLambda * (p.y - t.getOrigin().getY());
	pT.z = 0;
	return pT;
}

CoordTransformer::~CoordTransformer() {
}

