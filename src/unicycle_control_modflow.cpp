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
}

void ControlModule::setupNetwork ()
{
    requestConnection ("reference_source", &ControlModule::referenceSlot);
    requestConnection ("step_source", &ControlModule::stepSlot);

    _controlSink = requireSink<Tensor> ("publish_control");
}

void ControlModule::initParams (const NlParams &nlParams)
{
    _params = {
        .omegaGain = nlParams.get<float> ("omega_gain"),
        .targetVel = nlParams.get<float> ("target_vel"),
        .scaleFactor = nlParams.get<float> ("scale_factor")
    };
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
    int direction = 1;

    Tensor refComplex = torch::complex (_currentReference[0], _currentReference[1]);

	if (_currentReference.norm ().item ().toFloat () < 1e-6) {
		return torch::zeros ({2}, kFloat);
	}

    // Check if reference is behind the heading direction
    if (real (refComplex).lt (0).item().toBool ())  {
        // If so, reverse off
        refComplex = - refComplex;
        // and invert commands
        direction = -1;
    }

    Tensor diff = refComplex.log ();
    Tensor angleDiff = imag (diff);
    Tensor omega = _params.omegaGain * angleDiff;

    Tensor control = torch::empty ({2}, kFloat);

    control[0] = direction * _params.targetVel * exp(- omega.pow (2) / (2 * pow(M_PI * _params.scaleFactor, 2)));
    control[1] = omega;

    return control;
}

