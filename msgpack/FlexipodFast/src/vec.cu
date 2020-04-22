//
//  vec.cpp
//  CUDA Physics
//
//  Created by Jacob Austin on 5/13/18.
//  Copyright © 2018 Jacob Austin. All rights reserved.
//

#include "vec.h"


#if !defined(__CUDA_ARCH__) || __CUDA_ARCH__ >= 600

#else
//https://stackoverflow.com/questions/39274472/error-function-atomicadddouble-double-has-already-been-defined/39287554
static __inline__ __device__ double atomicDoubleAdd(double* address, double val) {
	unsigned long long int* address_as_ull = (unsigned long long int*)address;
	unsigned long long int old = *address_as_ull, assumed;
	if (val == 0.0)
		return __longlong_as_double(old);
	do {
		assumed = old;
		old = atomicCAS(address_as_ull, assumed, __double_as_longlong(val + __longlong_as_double(assumed)));
	} while (assumed != old);
	return __longlong_as_double(old);
}
#endif

// https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
// rotate a vector {v_} with rotation axis {k} anchored at point {offset} by {theta} [rad]
CUDA_CALLABLE_MEMBER Vec AxisAngleRotaion(const Vec& k, const Vec& v_, const double& theta, const Vec& offset) {
	Vec v = v_ - offset;
	double c = cos(theta);
	//Vec v_rot = v * c + cross(k, v) * sin(theta) + dot(k,v) * (1 - c) * k;
	Vec v_rot = cross(k, v);
	v_rot *= sin(theta);
	v_rot += v * c;
	v_rot += dot(k, v) * (1 - c) * k;
	v_rot += offset;
	return v_rot;
}

CUDA_DEVICE void Vec::atomicVecAdd(const Vec& v) {
#if defined(__CUDA_ARCH__) && __CUDA_ARCH__ >= 600
	atomicAdd(&x, v.x);
	atomicAdd(&y, v.y);
	atomicAdd(&z, v.z);
#elif defined(__CUDA_ARCH__) &&__CUDA_ARCH__ < 600
	atomicDoubleAdd(&x, v.x);
	atomicDoubleAdd(&y, v.y);
	atomicDoubleAdd(&z, v.z);
#endif
}