#ifndef UNICYCLE_CONTROL_H
#define UNICYCLE_CONTROL_H

#include "unicycle_control_modflow.h"

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

class UnicycleControlNode : public nlib::NlNode<UnicycleControlNode>
{
	NL_NODE(UnicycleControlNode)

	using ModFlow = UnicycleControlModFlow;

	struct Params {

	};

public:
	UnicycleControlNode (int &argc, char **argv, const std::string &name, uint32_t options = 0);

	void initROS ();
	void initParams ();

	void referenceCallback (const geometry_msgs::Pose2D &referenceMsg);
	void scanCallback (const sensor_msgs::LaserScan &scanMsg);
	void publishControl (const torch::Tensor &controlTensor);

	void stop ();

	DEF_SHARED(UnicycleControlNode)


protected:
	void onSynchronousClock (const ros::TimerEvent &timeEvent);

private:
	nlib::Channel _referenceChannel, _stepChannel, _obstacleChannel;
};

#endif // UNICYCLE_CONTROL_H
