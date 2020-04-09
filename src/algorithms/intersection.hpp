#pragma once

#include "pmp/MatVec.h"
#include <optional>

struct Ray
{
	pmp::vec3 origin;
	pmp::vec3 direction;
};

namespace algorithm {

	// Precomputed triangle structure to accelerate queries.
	struct IntersectionTriangle
	{
		IntersectionTriangle() = default;
		IntersectionTriangle(const pmp::vec3& p0, const pmp::vec3& p1, const pmp::vec3& p2);

		pmp::vec3 n0; float d0 = 0.f;
		pmp::vec3 n1; float d1 = 0.f;
		pmp::vec3 n2; float d2 = 0.f;
	};

	// Ray triangle intersection.
	std::optional<float> intersect(const Ray& ray, 
		const pmp::vec3& p0, 
		const pmp::vec3& p1, 
		const pmp::vec3& p2, 
		float maxDist);

	std::optional<float> intersect(const Ray& ray,
		const IntersectionTriangle& _triangle,
		float maxDist);

	std::optional<float> intersect(const Ray& ray,
		const pmp::vec3& p0,
		const pmp::vec3& n);
}