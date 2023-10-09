#ifndef UNICYCLE_CONTROL_MODFLOW_H
#define UNICYCLE_CONTROL_MODFLOW_H

#include <torch/all.h>
#include <nlib/nl_node.h>

class UnicycleControlModFlow : public nlib::NlModFlow
{
public:
	UnicycleControlModFlow ():
		 nlib::NlModFlow ()
	{}

	void loadModules () override;

	DEF_SHARED(UnicycleControlModFlow)
};

class ControlModule : public nlib::NlModule
{
public:
	struct Params {
		float omegaGain;
		float targetVel;
		float scaleFactor;
		float scaleFactorOffset;
		std::chrono::milliseconds reverseTimeThreshold_ms;
	};

    ControlModule (nlib::NlModFlow *modFlow);

	void initParams (const nlib::NlParams &nlParams) override;
	void setupNetwork () override;

	void referenceSlot (const torch::Tensor &reference);
	void obstacleSlot (float obstacle);
	void stepSlot();

	DEF_SHARED(ControlModule)

private:
	torch::Tensor getControl ();

private:
	bool lastReversed;
	std::chrono::time_point<std::chrono::system_clock> lastReversedStart;
	float _lastObstacle;
	Params _params;
    nlib::ReadyFlagsStr _flags;
	torch::Tensor _currentReference;
	nlib::Channel _controlSink;
};


#endif // UNICYCLE_CONTROL_MODFLOW_H
