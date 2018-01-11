#pragma once

#include "common.h"

namespace moper
{
	using vec3 = xo::vec3;
	using quat = xo::quat;

	struct CoordinateFrame
	{
		vec3 o;
		vec3 axes[3];
		quat q;
		void make_q() { q = xo::quat_from_axes( axes[ 0 ], axes[ 1 ], axes[ 2 ] ); }
	};

	inline vec3 transform_to( const CoordinateFrame& cf, const vec3& v )
	{
		return -cf.q * ( v - cf.o );
	}

	inline vec3 transform_from( const CoordinateFrame& cf, const vec3& v )
	{
		vec3 r = cf.o + ( cf.q * v );
		return r;
	}
}
