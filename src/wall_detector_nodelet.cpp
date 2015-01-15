/*
 * wall_detector_nodelet.h
 *
 *  Created on: Jan 15, 2015
 *      Author: biorob
 */

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "wall_detector.h"

class WallDetectorNodelet : public nodelet::Nodelet {
public:
	WallDetectorNodelet();
	virtual ~WallDetectorNodelet();

	virtual void onInit()
	  {
	    ros::NodeHandle nh = this->getPrivateNodeHandle();

	    // resolve node(let) name
	    std::string name = nh.getUnresolvedNamespace();
	    //NODELET_INFO_STREAM("Namespace " << name);
	    int pos = name.find_last_of('/');
	    name = name.substr(pos + 1);

	    NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
	    controller_.reset(new WallDetector(nh));

	  }
	private:
	  boost::shared_ptr<WallDetector> controller_;
};

PLUGINLIB_EXPORT_CLASS(WallDetectorNodelet,
                       nodelet::Nodelet);
