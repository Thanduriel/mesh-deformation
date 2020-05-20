#pragma once

#include <pmp/algorithms/DifferentialGeometry.h>
#include <utility>
#include <limits>
#include <iostream>

namespace util {

	std::pair<pmp::Face,float> find_min_angle(pmp::SurfaceMesh& mesh)
	{
		auto points = mesh.get_vertex_property<Point>("v:point");
	//	auto angles = mesh.add_face_property<float>("f:minangle");
		float minAngle = std::numeric_limits<float>::max();
		Face minFace;
		for (Face f : mesh.faces())
		{
			auto it = mesh.vertices(f).begin();
			const vec3& p0 = points[*it]; ++it;
			const vec3& p1 = points[*it]; ++it;
			const vec3& p2 = points[*it];

		//	const vec3 n = pmp::normalize(pmp::cross(p1 - p0, p2 - p0));
		//	if(n[2]>0)	std::cout << n << std::endl;

			float a = std::min(std::min(std::acos(pmp::cos(p1-p0, p2-p0)), 
				std::acos(pmp::cos(p0-p1, p2-p1))), std::acos(pmp::cos(p0 - p2,p1-p2)));
			if (a < minAngle)
			{
				minFace = f;
				minAngle = a;
			}
		}

		return { minFace, minAngle };
	}
}