#include <stdlib.h> 
class Deadzone_switch
{
public:
	Deadzone_switch(double* effort, double* model, double coefficient);
	double run();
private:
	double _coefficient;
	double* _effort;
	double* _model_output;
	bool _there_is_external_forces;

};
