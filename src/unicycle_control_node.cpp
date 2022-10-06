#include "../include/unicycle_control_node.h"

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
    addPub<geometry_msgs::Twist> ("cmd_vel", _nlParams.get<int> ("topics/queue_size", 1));
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

void UnicycleControlNode::onSynchronousClock (const ros::TimerEvent &timeEvent) {
	sources()->callSource (_stepChannel);
}

int main (int argc, char *argv[])
{
    UnicycleControlNode ucn(argc, argv, "unicycle_control");

    return ucn.spin ();
}



