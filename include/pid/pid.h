#ifndef CONTROL_SYSTEM_PID_PID_H
#define CONTROL_SYSTEM_PID_PID_H

#include <cstdint>
#include "math_lite.h"

using namespace math_lite::math_general;

namespace control_system
{
	namespace pid
	{
		template <typename T>
		class PID
		{
		public:
			PID();
			void init(T dt_, T Kp_, T Ki_, T Kd_, T u_k_1_, T u_max_);
			void set_param(T dt_, T Kp_, T Ki_, T Kd_, T u_k_1_, T u_max_);
			T update(T x_0, T x);
			void reset();
			void merge(T u_k_1_);
			void set_dt(T dt_);
			void set_Kp(T Kp_);
			void set_Ki(T Ki_);
			void set_Kd(T Kd_);
			void set_u_0(T u_k_1_);
			void set_u_max(T u_max_);
			T get_dt();
			T get_Kp();
			T get_Ki();
			T get_Kd();
			T get_u_k_1();
			T get_e_k_1();
			T get_e_k_2();
			T get_u_max();

		private:
			T dt = 0.0;
			T u_k_1 = 0.0;
			T e_k_1 = 0.0;
			T e_k_2 = 0.0;
			T Kp = 0.0;
			T Ki = 0.0;
			T Kd = 0.0;
			T u_max = 9999999999;
			uint8_t start = 0;
		};

#include "pid.tpp"
	}
}

#endif
