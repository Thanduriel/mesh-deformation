<img src="/mesh-deformation.png?raw=true">

## About
A small modeller for triangle meshes supporting multi-resolution editing and intuitive freeform modifications based on [\[1\]](#references). Technical details regarding the implementation and user controllable parameters are explained in [docs/report](docs/report.pdf).

Some example meshes can be found in `models/` and `dependencies/pmp-library/external/pmp-data`.
Supported formats include `off`,`obj` and `stl`, for the full list see the  [pmp-library documentation](http://www.pmp-library.org/classpmp_1_1_surface_mesh.html#ab76e687f8e2ba74cb6d5918fc43caf36).

## Build
To build the project just clone the repo with submodules:

    git clone --recursive https://github.com/Thanduriel/mesh-deformation.git
	cd mesh-deformation
	mkdir build
	cd build
	cmake ..

Tested with gcc 8.1 and MSVC 2019.

## Getting Started

Press `?` at any time to open up the help menu.

The program has two different modes: drawing and operator mode.
After one mesh is loaded, you can use your left mouse button to rotate the mesh and your scroll wheel (or right mouse button) to zoom in and out.
To draw the regions you can either use the dropdown menu or the `W`, `Q` or `E` button on your keyboard.

Remark: If you have selected one of the draw modes above, you can't rotate the mesh furthermore. For this you need to go back to the view mode. (button `R` or change the selection via dropdown)

	W: Draws support region
	Q: Draws handle region
	E: Clears region
	R: Go back into the view mode
	A: Grow current region
	S: Shrink current region
	BACKSPACE: Reload mesh

After you marked some region on your mesh, press `SPACE` to go into the operator mode. In this mode you can't draw any more regions on your mesh. For this go back into the draw mode by pressing the `SPACE` button again.

In the operator mode you can see further modification settings with regard to the deformation algorithm. Like order of the `la place operator` or the `smoothness` factor.

There are three ways in which you can deform the mesh. First one is the `translation mode` where you can move the handle region along an local axis. Second is the `rotation mode` to rotate around an local axis and the last one is the `scale mode` for scaling the handle region.

	SPACE: Switch to operator mode or back to the draw mode
	T: Switch to translation mode
	S: Switch to scale mode
	R: Switch to rotation mode

## References
\[1\] [Botsch, M., & Kobbelt, L. (2004). An intuitive framework for real-time freeform modeling. ACM Transactions on Graphics (TOG), 23(3), 630-634](https://graphics.uni-bielefeld.de/downloads/publications/2004/sg04.pdf)