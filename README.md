<img src="/mesh-deformation.png?raw=true">

## About
A small modeller for triangle meshes supporting multi-resolution editing and intuitive freeform modifications based on [\[1\]](#references).
Some example meshes can be found in `models/` and `dependencies/pmp-library/external/pmp-data`.
Supported formats include `off`,`obj` and `stl`, for the full list see the  [pmp-library documentation](http://www.pmp-library.org/classpmp_1_1_surface_mesh.html#ab76e687f8e2ba74cb6d5918fc43caf36).

## Build
To build the project just clone the repo with submodules:

    git clone --recursive https://github.com/Thanduriel/mesh-deformation.git
	mkdir build
	cd build
	cmake ..

Tested with gcc 8.1 and MSVC 2019.

## References
\[1\] [Botsch, M., & Kobbelt, L. (2004). An intuitive framework for real-time freeform modeling. ACM Transactions on Graphics (TOG), 23(3), 630-634](https://graphics.uni-bielefeld.de/downloads/publications/2004/sg04.pdf)