#include "p.h"

template <typename T>
control_system::proportional::P<T>::P()
{
	Kp = 0.0;
}

template <typename T>
void control_system::proportional::P<T>::init(T Kp_, T u_max_)
{
	set_param(Kp_, u_max_);
}

template <typename T>
void control_system::proportional::P<T>::set_param(T Kp_, T u_max_)
{
	Kp = Kp_;
	u_max = u_max_;
}

template <typename T>
T control_system::proportional::P<T>::update(T x_0, T x)
{
	return saturate(Kp * (x_0 - x), -u_max, u_max);
}

template <typename T>
void control_system::proportional::P<T>::reset()
{
}

template <typename T>
void control_system::proportional::P<T>::set_Kp(T Kp_)
{
	Kp = Kp_;
}

template <typename T>
void control_system::proportional::P<T>::set_u_max(T u_max_)
{
	u_max = u_max_;
}

template <typename T>
T control_system::proportional::P<T>::get_Kp()
{
	return Kp;
}

template <typename T>
T control_system::proportional::P<T>::get_u_max()
{
	return u_max;
}
