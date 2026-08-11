#ifndef CONTROL_SYSTEM_PD_PD_H
#define CONTROL_SYSTEM_PD_PD_H

#include "math_lite.h"

using namespace math_lite::math_general;

namespace control_system
{
	namespace pd
	{
		template <typename T>
		class PD
		{
		public:
			PD();
			void init(T dt_, T Kp_, T Kd_, T u_max_);
			void set_param(T dt_, T Kp_, T Kd_, T u_max_);
			T update(T x_0, T x);
			void reset();
			void merge(T u_k_1_);
			void set_dt(T dt_);
			void set_Kp(T Kp_);
			void set_Kd(T Kd_);
			void set_u_max(T u_max_);
			T get_dt();
			T get_Kp();
			T get_Kd();
			T get_e_k_1();
			T get_u_max();

		private:
			T dt = 0.0;
			T e_k_1 = 0.0;
			T Kp = 0.0;
			T Kd = 0.0;
			bool start = true;
			T u_max = 9999999999;
		};

#include "pd.tpp"
	}
}

#endif
