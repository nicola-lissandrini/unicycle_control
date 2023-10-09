#include "../include/unicycle_control_node.h"

#include <signal.h>

using namespace std;
using namespace nlib;
using namespace torch;

void pose2DToTensor (const geometry_msgs::Pose2D referenceMsg, torch::Tensor &referenceTensor) {
	referenceTensor[0] = referenceMsg.x;
	referenceTensor[1] = referenceMsg.y;
	referenceTensor[2] = referenceMsg.theta;
}

void controlTensorToTwist (const torch::Tensor &controlTensor, geometry_msgs::Twist &controlMsg) {
	controlMsg.linear.x = controlTensor[0].item().toFloat ();
	controlMsg.angular.z = controlTensor[1].item().toFloat ();
}


UnicycleControlNode::UnicycleControlNode (int &argc, char **argv, const string &name, uint32_t options):
	 Base (argc, argv, name, options)
{
	init<ModFlow> ();

    _referenceChannel = sources()->declareSource<Tensor> ("reference_source");
    _obstacleChannel = sources()->declareSource<float> ("obstacle_source");
	_stepChannel = sources()->declareSource<> ("step_source");

    sinks()->declareSink ("publish_control", &UnicycleControlNode::publishControl, this);

	finalizeModFlow ();

}

void UnicycleControlNode::initParams ()
{

}

void UnicycleControlNode::initROS ()
{
    addSub ("reference", _nlParams.get<int> ("topics/queue_size", 1), &UnicycleControlNode::referenceCallback);
    addSub ("scan", 1, &UnicycleControlNode::scanCallback);
    addPub<geometry_msgs::Twist> ("cmd_vel", _nlParams.get<int> ("topics/queue_size", 1));
}

void UnicycleControlNode::scanCallback (const sensor_msgs::LaserScan &scanMsg) {
	const auto &ranges = scanMsg.ranges;

	float obstacle = *std::min_element (ranges.begin (), ranges.end ());

	sources()->callSource (_obstacleChannel, obstacle);
}

void UnicycleControlNode::referenceCallback (const geometry_msgs::Pose2D &referenceMsg) {
	Tensor currentReference = torch::empty ({3}, kFloat);
	pose2DToTensor (referenceMsg, currentReference);
	sources()->callSource (_referenceChannel, currentReference);
}

void UnicycleControlNode::publishControl (const Tensor &controlTensor) {
	geometry_msgs::Twist controlMsg;

	controlTensorToTwist (controlTensor, controlMsg);

	publish ("cmd_vel", controlMsg);
}

void UnicycleControlNode::stop() {
	publish ("cmd_vel", geometry_msgs::Twist ());
}

void UnicycleControlNode::onSynchronousClock (const ros::TimerEvent &timeEvent) {
	sources()->callSource (_stepChannel);
}

UnicycleControlNode *ucnPtr;

void sigint (int sig) {
	ucnPtr->stop ();

	ros::shutdown ();
}

int main (int argc, char *argv[])
{
	ucnPtr = new UnicycleControlNode (argc, argv, "unicycle_control");

	signal (SIGINT, sigint);

	return ucnPtr->spin ();
}



