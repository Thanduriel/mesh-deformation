#include "intersection.hpp"

namespace algorithm {

	using namespace pmp;

	// taken from https://stackoverflow.com/a/44837726

	struct isect_hh_data 
	{
		vec3 n0; float d0;
		vec3 n1; float d1;
		vec3 n2; float d2;
	};

	static isect_hh_data isect_hh_pre(vec3 v0, vec3 v1, vec3 v2)
	{
		vec3 e1 = v1 - v0;
		vec3 e2 = v2 - v0;
		isect_hh_data D;
		D.n0 = cross(e1, e2);
		D.d0 = dot(D.n0, v0);

		float inv_denom = 1 / dot(D.n0, D.n0);

		D.n1 = cross(e2, D.n0) * inv_denom;
		D.d1 = -dot(D.n1, v0);

		D.n2 = cross(D.n0, e1) * inv_denom;
		D.d2 = -dot(D.n2, v0);

		return D;
	}

	inline int float_to_int_cast(float f)
	{
		return *reinterpret_cast<int*>(&f);
	}

	static bool isect_hh(vec3 o, vec3 d, float* t, float maxDist, const isect_hh_data& D) 
	{
		vec2 uv;

		float det = dot(D.n0, d);
		float dett = D.d0 - dot(o, D.n0);
		vec3 wr = o * det + d * dett;
		uv[0] = dot(wr, D.n1) + det * D.d1;
		uv[1] = dot(wr, D.n2) + det * D.d2;
		float tmpdet0 = det - uv[0] - uv[1];
		int pdet0 = float_to_int_cast(tmpdet0);
		int pdetu = float_to_int_cast(uv[0]);
		int pdetv = float_to_int_cast(uv[1]);
		pdet0 = pdet0 ^ pdetu;
		pdet0 = pdet0 | (pdetu ^ pdetv);
		if (pdet0 & 0x80000000)
			return false;
		float rdet = 1 / det;
		uv[0] *= rdet;
		uv[1] *= rdet;
		*t = dett * rdet;
		return true;//*t >= 0.f && *t <= maxDist;
	}

	std::optional<float> intersect(const Ray& ray, 
		const pmp::vec3& p0, const pmp::vec3& p1, const pmp::vec3& p2, float maxDist)
	{
		const isect_hh_data triangle = isect_hh_pre(p0, p1, p2);
		float dist;
		return isect_hh(ray.origin, ray.direction, &dist, maxDist, triangle)
			? std::optional<float>(dist) : std::nullopt;
		
	}
}