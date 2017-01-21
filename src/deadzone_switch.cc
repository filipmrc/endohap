#include <endohap/deadzone_switch.h>


Deadzone_switch::Deadzone_switch(double* effort, double* model, double coefficient) :
	_coefficient(coefficient),
	_effort(effort),
	_model_output(model)
{
	_there_is_external_forces = false;
};



double Deadzone_switch::run()
{
	double output = 0.0;
	if(abs(*_effort) > _coefficient)
		_there_is_external_forces = true;

	if(_there_is_external_forces)
	{
		output = *_model_output;
		if(abs(*_model_output) < _coefficient)
			_there_is_external_forces = false;
	}
	return output;
}

