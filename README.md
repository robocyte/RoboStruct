RoboStruct
==========

This program implements a full structure from motion pipeline, from detecting and matching image features
to computing 3d structure and camera positions.

It's based on Noah Snavely's bundler (http://www.cs.cornell.edu/~snavely/bundler/).
The original code has changed quite a bit, mainly by using OpenCV for feature detection, matching and
homography/fundamental matrix estimation. Furthermore, Eigen is used for linear algebra,
SBA was dropped in favor of Ceres for bundle adjustment.
WxWidgets is used for the GUI, glm and GLEW for OpenGL related stuff. You can find a complete list
of used libraries under compiling.


Usage
=====

The program is still unfinished but a working executable (Windows, 64 bit) can be found in /bin.
Just put the /bin folder anywhere on your harddrive. Select another folder containing .jpg files and hit the big play button, et voila.
Important: jpg. files must contain the focal length information!
Depending on your computer's memory and speed, it may be advisable to resize the source images.
For reference, on my system (Core i7 quadcore, 16 GB Ram), image sizes up to 3000 x 2000 pixels are tested and work well.

As with many other feature based sfm programs it is recommended to take pictures with wide angle lenses and a
sufficient but not too large baseline. Naturally, rich textured, uniformely lit and non-reflective surfaces are
optimal candidates for reconstruction.

In order to see something in the 3d viewport, a graphics card capable of OpenGL 3.3 and above is needed.

Finally, results can be exported as .ply and .out files (from bundler), also exporting to MAYA and CMVS is possible.


Compiling
=========

Building the program from source is not possible yet because my OpenGL framework (glocyte) is not released.
This will change soon, so stay tuned.

As for the rest of the dependencies, here is the list:

wxWidgets    http://www.wxwidgets.org/
OpenCV       http://opencv.org/
Ceres        http://code.google.com/p/ceres-solver/
Eigen        http://eigen.tuxfamily.org/index.php?title=Main_Page
Daisy        http://cvlab.epfl.ch/software/daisy
OpenCTM      http://openctm.sourceforge.net/
glm          http://glm.g-truc.net/0.9.4/index.html
GLEW         http://glew.sourceforge.net/
glocyte      coming soon...

The program is currently developed for Windows, a Visual Studio 2013 solution is provided.
