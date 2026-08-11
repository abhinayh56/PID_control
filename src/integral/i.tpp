#include "i.h"

template <typename T>
control_system::integral::I<T>::I()
{
	dt = 0.0;
	u_k_1 = 0.0;
	Ki = 0.0;
}

template <typename T>
void control_system::integral::I<T>::init(T dt_, T Ki_, T u_k_1_, T u_max_)
{
	set_param(dt_, Ki_, u_k_1_, u_max_);
}

template <typename T>
void control_system::integral::I<T>::set_param(T dt_, T Ki_, T u_k_1_, T u_max_)
{
	dt = dt_;
	Ki = Ki_;
	u_k_1 = u_k_1_;
	u_max = u_max_;
}

template <typename T>
T control_system::integral::I<T>::update(T x_0, T x)
{
	T e_k = x_0 - x;
	T u_k = u_k_1 + Ki * dt * e_k;
	u_k = saturate(u_k, -u_max, u_max);
	u_k_1 = u_k;
	return u_k;
}

template <typename T>
void control_system::integral::I<T>::reset()
{
	u_k_1 = 0.0;
}

template <typename T>
void control_system::integral::I<T>::merge(T u_k_1_)
{
	u_k_1 = u_k_1_;
}

template <typename T>
void control_system::integral::I<T>::set_dt(T dt_)
{
	dt = dt_;
}

template <typename T>
void control_system::integral::I<T>::set_Ki(T Ki_)
{
	Ki = Ki_;
}

template <typename T>
void control_system::integral::I<T>::set_u_0(T u_k_1_)
{
	u_k_1 = u_k_1_;
}

template <typename T>
void control_system::integral::I<T>::set_u_max(T u_max_)
{
	u_max = u_max_;
}

template <typename T>
T control_system::integral::I<T>::get_dt()
{
	return dt;
}

template <typename T>
T control_system::integral::I<T>::get_Ki()
{
	return Ki;
}

template <typename T>
T control_system::integral::I<T>::get_u_k_1()
{
	return u_k_1;
}

template <typename T>
T control_system::integral::I<T>::get_u_max()
{
	return u_max;
}
