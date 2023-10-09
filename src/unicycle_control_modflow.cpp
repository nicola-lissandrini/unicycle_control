#include "../include/unicycle_control_modflow.h"

using namespace std;
using namespace nlib;
using namespace torch;

void UnicycleControlModFlow::loadModules () {
    loadModule<ControlModule> ();
}

ControlModule::ControlModule(nlib::NlModFlow *modFlow):
    nlib::NlModule (modFlow, "control")
{
    _flags.addFlag ("first_reference");

	lastReversed = false;
}

void ControlModule::setupNetwork ()
{
    requestConnection ("reference_source", &ControlModule::referenceSlot);
    requestConnection ("step_source", &ControlModule::stepSlot);
    requestConnection ("obstacle_source", &ControlModule::obstacleSlot);

    _controlSink = requireSink<Tensor> ("publish_control");
}

void ControlModule::initParams (const NlParams &nlParams)
{
    _params = {
        .omegaGain = nlParams.get<float> ("omega_gain"),
        .targetVel = nlParams.get<float> ("target_vel"),
        .scaleFactor = nlParams.get<float> ("scale_factor"),
        .scaleFactorOffset = nlParams.get<float> ("scale_factor_offset"),
		.reverseTimeThreshold_ms = std::chrono::milliseconds (nlParams.get<int> ("reverse_time_threshold_ms"))
    };
}

void ControlModule::obstacleSlot (float obstacle) {
    _lastObstacle = obstacle;
}

void ControlModule::referenceSlot (const Tensor &reference) {
    _flags.set ("first_reference");
    _currentReference = reference;
}

void ControlModule::stepSlot ()
{
    if (!_flags.all())
        return;

    Tensor control = getControl ();

    emit (_controlSink, control);
}

Tensor ControlModule::getControl ()
{
	// Currently use locobot in reverse
	int direction = -1;

	Tensor refComplex = -torch::complex (_currentReference[0], _currentReference[1]);


	if (_currentReference.norm ().item ().toFloat () < 1e-6)
		return torch::zeros ({2}, kFloat);
	
	

	if (real (refComplex).lt (0).item ().toBool ()) {
		if (!lastReversed)
			lastReversedStart = std::chrono::system_clock::now ();

		lastReversed = true;

		if (std::chrono::system_clock::now () - lastReversedStart < _params.reverseTimeThreshold_ms) {
			refComplex = - refComplex;
			direction = 1;
		}
	} else {
		lastReversed = false;
	}

	COUTN(refComplex);

    Tensor diff = refComplex.log ();
    Tensor angleDiff = imag (diff);
    Tensor omega = _params.omegaGain * angleDiff;

    Tensor control = torch::empty ({2}, kFloat);

    control[0] = direction * _params.targetVel * exp(- omega.pow (2) / (2 * pow(M_PI * _params.scaleFactor * (_lastObstacle - _params.scaleFactorOffset), 2))); 
    control[1] = omega;

    return control;
}

