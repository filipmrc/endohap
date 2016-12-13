void saturation(double* force, double bound)
{
	if (*force > bound) *force = bound;
	if (*force < -bound) *force = -bound;
	if (*force != *force) *force = 0.0;
}
int signum(double* d)
{
	return (*d > 0) - (*d < 0);
}
void deadzone(double* value, double bound)
{
	if (abs(*value) < bound)
		*value = 0.0;
	else
		*value = *value + (signum(value)*bound);
}
