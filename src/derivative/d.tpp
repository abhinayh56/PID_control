#include "d.h"

template <typename T>
control_system::derivative::D<T>::D()
{
	dt = 0.0;
	e_k_1 = 0.0;
	Kd = 0.0;
	start = true;
}

template <typename T>
void control_system::derivative::D<T>::init(T dt_, T Kd_, T u_max_)
{
	set_param(dt_, Kd_, u_max_);
	start = true;
}

template <typename T>
void control_system::derivative::D<T>::set_param(T dt_, T Kd_, T u_max_)
{
	dt = dt_;
	Kd = Kd_;
	u_max = u_max_;
}

template <typename T>
T control_system::derivative::D<T>::update(T x_0, T x)
{
	T e_k = x_0 - x;
	T u_k = 0.0;
	if (start == true)
	{
		start = false;
		u_k = 0.0;
	}
	else
	{
		u_k = Kd * (e_k - e_k_1) / dt;
	}
	u_k = Math_general<T>::saturate(u_k, -u_max, u_max);
	e_k_1 = e_k;
	return u_k;
}

template <typename T>
void control_system::derivative::D<T>::reset()
{
	e_k_1 = 0.0;
	start = true;
}

template <typename T>
void control_system::derivative::D<T>::merge(T u_k_1_)
{
}

template <typename T>
void control_system::derivative::D<T>::set_dt(T dt_)
{
	dt = dt_;
}

template <typename T>
void control_system::derivative::D<T>::set_Kd(T Kd_)
{
	Kd = Kd_;
}

template <typename T>
void control_system::derivative::D<T>::set_u_max(T u_max_)
{
	u_max = u_max_;
}

template <typename T>
T control_system::derivative::D<T>::get_dt()
{
	return dt;
}

template <typename T>
T control_system::derivative::D<T>::get_Kd()
{
	return Kd;
}

template <typename T>
T control_system::derivative::D<T>::get_e_k_1()
{
	return e_k_1;
}
