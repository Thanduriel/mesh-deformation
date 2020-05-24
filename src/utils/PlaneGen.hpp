#pragma once

#include <pmp/SurfaceMesh.h>

namespace util {
	class PlaneGenerator
	{
	public:
		pmp::SurfaceMesh generate(pmp::vec2 size, pmp::ivec2 numPoints, float noise = 0.f);
	};

}