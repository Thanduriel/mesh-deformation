#pragma once

#include "pmp/MatVec.h"
#include <optional>

struct Ray
{
	pmp::vec3 origin;
	pmp::vec3 direction;
};

namespace algorithm {

	std::optional<float> intersect(const Ray& ray, 
		const pmp::vec3& p0, 
		const pmp::vec3& p1, 
		const pmp::vec3& p2, 
		float maxDist);
}