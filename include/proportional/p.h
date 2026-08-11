#ifndef CONTROL_SYSTEM_PROPORTIONAL_P_H
#define CONTROL_SYSTEM_PROPORTIONAL_P_H

#include "../utility/utility.h"

namespace control_system
{
	namespace proportional
	{
		template <typename T>
		class P
		{
		public:
			P();
			void init(T Kp_, T u_max_);
			void set_param(T Kp_, T u_max_);
			T update(T x_0, T x);
			void reset();
			void set_Kp(T Kp_);
			void set_u_max(T u_max_);
			T get_Kp();
			T get_u_max();

		private:
			T Kp = 0.0;
			T u_max = 9999999999;
		};

#include "p.tpp"
	}
}

#endif
