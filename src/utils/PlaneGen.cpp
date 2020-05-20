#include "PlaneGen.hpp"
#include <random>

namespace util {

	using namespace pmp;

	SurfaceMesh PlaneGenerator::generate(vec2 size, ivec2 numPoints, float noise)
	{
		SurfaceMesh mesh;

		std::vector<Vertex> vertices;
		vertices.reserve(numPoints[0] * numPoints[1]);
		const vec2 stepSize(size[0] / numPoints[0], size[1] / numPoints[1]);

		std::default_random_engine rng;
		std::uniform_real_distribution<float> distX(-stepSize[0] * noise, stepSize[0] * noise);
		std::uniform_real_distribution<float> distY(-stepSize[1] * noise, stepSize[1] * noise);

		for (int iy = 0; iy < numPoints[0]; ++iy)
		{
			const float y = iy * stepSize[1];
			for (int ix = 0; ix < numPoints[1]; ++ix)
			{
				const float x = ix * stepSize[0];
				vertices.push_back(mesh.add_vertex(Point(x + distX(rng), y + distY(rng), 0.f)));
			}
		}

		for (int iy = 0; iy < numPoints[0]-1; ++iy)
		{
			for (int ix = 0; ix < numPoints[1]-1; ++ix)
			{
				const Vertex v1 = vertices[ix + iy * numPoints[0]];
				const Vertex v2 = vertices[ix + 1 + iy * numPoints[0]];
				const Vertex v3 = vertices[ix + (iy + 1) * numPoints[0]];
				const Vertex v4 = vertices[ix + 1 + (iy + 1) * numPoints[0]];
				mesh.add_triangle(v1,v3,v2);
				mesh.add_triangle(v4,v2,v3);
			}
		}

		return mesh;
	}
}