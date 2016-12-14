#ifndef UTILS_H
#define UTILS_H

static void saturation(double* force, double bound)
{
	if (*force > bound) *force = bound;
	if (*force < -bound) *force = -bound;
	if (*force != *force) *force = 0.0;
}
static int signum(double d)
{
	return (d > 0) - (d < 0);
}
static double deadzone(double value, double bound)
{
	if (std::abs(value) < bound)
		return 0.0;
	else
		return value - (signum(value)*bound);
}
#endif
