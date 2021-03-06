#include "Intersection.hpp"
#include <limits>
#include <cstring>

namespace algorithm {

	using namespace pmp;

	// taken from https://stackoverflow.com/a/44837726

	IntersectionTriangle::IntersectionTriangle(const vec3& v0, const vec3& v1, const vec3& v2)
	{
		vec3 e1 = v1 - v0;
		vec3 e2 = v2 - v0;

		n0 = cross(e1, e2);
		d0 = dot(n0, v0);

		float inv_denom = 1 / dot(n0, n0);

		n1 = cross(e2, n0) * inv_denom;
		d1 = -dot(n1, v0);

		n2 = cross(n0, e1) * inv_denom;
		d2 = -dot(n2, v0);
	}

	inline int float_to_int_cast(float f)
	{
		static_assert(sizeof(float) == sizeof(int));

		// std conform reinterpret cast
		int res;
		std::memcpy(&res, &f, sizeof(float));
		return res;
	}

	static bool intersect(vec3 o, vec3 d, float* t, const IntersectionTriangle& D)
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
		return true;
	}

	std::optional<float> intersect(const Ray& ray, 
		const pmp::vec3& p0, const pmp::vec3& p1, const pmp::vec3& p2)
	{
		const IntersectionTriangle triangle(p0, p1, p2);
		
		return intersect(ray, triangle);	
	}

	std::optional<float> intersect(const Ray& ray,
		const IntersectionTriangle& triangle)
	{
		float dist;
		return intersect(ray.origin, ray.direction, &dist, triangle)
			? std::optional<float>(dist) : std::nullopt;

	}

	std::optional<float> intersect(const Ray& ray, const vec3& p0, const vec3& n)
	{
		const float den = dot(ray.direction, n);
		// almost parallel
		if (std::abs(den) < std::numeric_limits<float>::epsilon() * 16.f)
			return std::nullopt;

		return dot(p0 - ray.origin, n) / den;
	}
}
