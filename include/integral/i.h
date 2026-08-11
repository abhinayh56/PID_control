#ifndef CONTROL_SYSTEM_INTEGRAL_I_H
#define CONTROL_SYSTEM_INTEGRAL_I_H

#include "../utility/utility.h"

namespace control_system
{
	namespace integral
	{
		template <typename T>
		class I
		{
		public:
			I();
			void init(T dt_, T Ki_, T u_k_1_, T u_max_);
			void set_param(T dt_, T Ki_, T u_k_1_, T u_max_);
			T update(T x_0, T x);
			void reset();
			void merge(T u_k_1_);
			void set_dt(T dt_);
			void set_Ki(T Ki_);
			void set_u_0(T u_k_1_);
			void set_u_max(T u_max_);
			T get_dt();
			T get_Ki();
			T get_u_k_1();
			T get_u_max();

		private:
			T dt = 0.0;
			T u_k_1 = 0.0;
			T Ki = 0.0;
			T u_max = 9999999999;
		};

#include "i.tpp"
	}
}

#endif
