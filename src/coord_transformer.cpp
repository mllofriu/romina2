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
			// points x must be close enough, if not: vertical line in the image
			if (p1.x > 0 && p2.x > 0/* && lenght(p1, p2) < LINE_LEN_THRS*/ ) {
				// Extend the line to account for possible unseen parts of it
				Point32 p1Ext, p2Ext;
				extendLine(p1, p2,pclOut.points.at(i).x, pclOut.points.at(i+1).x,  LINE_EXTEND_FACTOR, imgW, p1Ext, p2Ext);
				pol.points.push_back(p1Ext);
				pol.points.push_back(p2Ext);

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

/**
 * Extend a line factor times around the middle point
 */
void CoordTransformer::extendLine(Point32 p1, Point32 p2,int imgX1, int imgX2,float factor, int img_width,  Point32 & p1Ext,
		Point32 & p2Ext) {
	float x1 = p1.x;
	float x2 = p2.x;
	float y1 = p1.y;
	float y2 = p2.y;
	float lambda = ((float) (y2 - y1)) / (x2 - x1);
	// Extend only the points whose x in the image is close to the boundary
	if (imgX1 <= IMG_BOUNDARY_THRS || imgX1 >= img_width - IMG_BOUNDARY_THRS) {
		p1Ext.x = x1 + (x2 - x1) / 2 - factor * (x2 - x1);
		p1Ext.y = y1 + (y2 - y1) / 2 - factor * lambda * (x2 - x1);
	} else {
		p1Ext.x = p1.x;
		p1Ext.y = p1.y;
	}
	if (imgX2 <= IMG_BOUNDARY_THRS || imgX2 >= img_width - IMG_BOUNDARY_THRS) {
		p2Ext.x = x1 + (x2 - x1) / 2 + factor * (x2 - x1);
		p2Ext.y = y1 + (y2 - y1) / 2 + factor * lambda * (x2 - x1);
	} else {
		p2Ext.x = p2.x;
		p2Ext.y = p2.y;
	}
}

float CoordTransformer::lenght(Point32 & p1, Point32 & p2) {
	return sqrt(pow((float) p1.x - p2.x, 2) + pow((float) p1.y - p2.y, 2));
}

CoordTransformer::~CoordTransformer() {
}

