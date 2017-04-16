The Structured Contact Impact Simulator
=======================================
[![Build Status](https://travis-ci.org/breannansmith/scisim.svg?branch=master)](https://travis-ci.org/breannansmith/scisim)
[![Circle CI](https://circleci.com/gh/breannansmith/scisim.svg?style=svg)](https://circleci.com/gh/breannansmith/scisim)

Welcome to **SCISim** (pronounced skiz-em), the Structured Contact Impact Simulator! **SCISim** is a C++14 software package designed to simulate systems of rigid bodies where the evolution is dominated by collisions. **SCISim** respects essential symmetries and structures from the continuous setting at coarse temporal discretizations through the use of geometric integrators for the unconstrained and constrained dynamics. In practice, this leads to stable and predictable behavior across timesteps. For a preview of the types of simulations that **SCISim** can perform, please see our demo reel. (Coming soon. Until then, please see videos from a [few](https://www.youtube.com/watch?v=AJAGUOhpnLc) [papers](https://www.youtube.com/watch?v=tFKLo0yNmFk) that use **SCISim**.)

To resolve collisions between volumetric objects, SCISim employs precomputed implicit distance fields. We provide a tool to compute these distance fields from a triangle mesh in a [separate repository](https://github.com/breannansmith/implicittoolkit).

Required Dependencies
---------------------

SCISim requires three dependencies for a minimal build:

* [RapidXML](http://rapidxml.sourceforge.net/): An XML parser to read simulation descriptions.

* [Eigen](http://eigen.tuxfamily.org/): A linear algebra library used internally.

* [So-bogus](https://bitbucket.org/gdaviet/so-bogus): A non-smooth Coulomb friction solver.

We provide a 'get_dependencies.sh' script to automatically download, verify, and extract the supported versions of these libraries.


Recommended Dependencies
------------------------

We recommend a few dependencies for full featured builds:

* [Qt5](https://www.qt.io/): A user interface library to provide graphical front ends. Available through most standard package managers.

* [HDF5](https://www.hdfgroup.org/HDF5/): A binary file format for configuration and force output. Available through most standard package managers.

* [Ipopt](https://projects.coin-or.org/Ipopt): A nonlinear optimization package for global LCP solves, for the Staggered Projections friction solver, and for the Generalized Reflections and Generalized Restitution impact models. Ipopt requires a Fortran compiler. See the included [Ipopt build instructions](readme_ipopt.md).

* [HSL2013](http://www.hsl.rl.ac.uk/ipopt/): A collection of sparse linear solvers suggested for use with Ipopt.

* [Python](https://www.python.org): An interpreted language used for extending SCISim's behavior with plugins. Python is primarily used to script the motion of kinematic bodies and to set custom parameters in the contact model. If these fall outside your intended use case, you can safely omit the Python dependency. Available standard on most platforms. Note that full SCISim test suite requires the installation of the [numpy](http://www.numpy.org) and [h5py](http://www.h5py.org) Python packages.

Optional Dependencies
---------------------

SCISim optionally supports other numerical solvers:

* [QL](http://www.ai7.uni-bayreuth.de/ql.htm): A quadratic program solver useful for small to medium problems. To use QL, place the file ql.for under SCISim/Math/QL and rebuild. QL requires a Fortran compiler, and is available for academic use.


Quickstart Guide
----------------

To obtain a minimal demo build that simulates colliding triangle meshes:

1. Install Qt5, HDF5, and CMake. These packages are available from most standard package managers.

2. Clone this repository and change into the project root:

        git clone https://github.com/breannansmith/scisim.git
        cd scisim

3. From the project root, run the script get_dependencies.sh to download, extract, and verify the required dependencies. Note that this script requires the md5 or md5sum utility:

        ./get_dependencies.sh

4. Create a build directory under the project root and change into this directory:

        mkdir build
        cd build

5. Run CMake to create the build system with Qt5 and HDF5 enabled. Note that you might need to add the Qt5 installation path to the CMAKE_PREFIX_PATH variable (for example, with MacPorts, append 'CMAKE_PREFIX_PATH="/usr/local/opt/qt"' to the following command):

        cmake -DUSE_QT5=ON -DUSE_HDF5=ON ..

6. Build SCISim:

        make

7. Run the example simulation:

    1. With *nix:

            cd rigidbody3dqt5
            ./rigidbody3d_qt5 assets/tests_serialization/two_dragon_drop.xml

    2. With macOS:

            cd rigidbody3dqt5
            ./rigidbody3d_qt5.app/Contents/MacOS/rigidbody3d_qt5 assets/tests_serialization/two_dragon_drop.xml

8. Press space to run the simulation!


Building SCISim
---------------

SCISim uses the CMake build system. SCISim is tested regularly against recent versions of the GCC, LLVM, and Intel compiler toolchains on both Linux and macOS. A minimal SCISim build requires only a C and C++ compiler. The optional QL and Ipopt dependencies require an additional Fortran compiler.

As an example, to build SCISim with an optional Fortran compiler using the Intel toolchain, from a build directory, run:

    CC=icc CXX=icpc FC=ifort cmake ..

Substitute out the compiler names as required by your platform. The build system can be configured via the command line by running

    ccmake ..

from the build directory.

Options of note include:

* CMAKE_BUILD_TYPE: General build type that enables various optimization and compiler flags. Options are: Debug, Release, RelWithDebInfo, MinSizeRel

* STRICT_BUILD: Enables aggressive warnings and treats warnings as errors. Recommended for development.

* USE_HDF5: Enables state and force output via [HDF5](https://www.hdfgroup.org/HDF5/) files.

* USE_IPOPT: Enables the [Ipopt](https://projects.coin-or.org/Ipopt) nonlinear solver.

* USE_OPENMP: Enables OpenMP support. Highly recommend for the [So-bogus](https://bitbucket.org/gdaviet/so-bogus) solver.

* USE_QT5: Enables support for graphical front ends using [Qt5](https://www.qt.io/).

* USE_PYTHON: Enables support for embedded Python language scripting. Required for kinematic scripting.

* SANITIZER: Enables support for compiler sanitizer modes. Options are: none, address, undefined

To enable the build system to find Ipopt, prefix the cmake command with CMAKE_PREFIX_PATH set to your Ipopt [installation directory](https://github.com/breannansmith/scisim/blob/master/readme_ipopt.md):

    CMAKE_PREFIX_PATH=/path/to/ipopt/install CC=gcc CXX=g++ FC=gfortran cmake -DUSE_IPOPT=ON ..

Platform Specific Issues
------------------------

* macOS
  * There are performance regressions with GCC and So-bogus when building with the GCC toolchain provided by MacPorts.
  * There are a number of strange behaviors with the Qt5 frontend when building against the version from MacPorts. I intend to upgrade to Qt5, which should remedy these issues.
  * Building Ipopt with GCC is incompatible with building SCISim with Clang.
  * CMake will often pull in different versions of the Python interpreter and the Python library, requiring the user to explicitly pass the location of the Python library to CMake. For example, to build with MacPorts' Python 2.7:

            cmake -DPYTHON_LIBRARY=/opt/local/lib/libpython2.7.dylib -DPYTHON_INCLUDE_DIR=/opt/local/Library/Frameworks/Python.framework/Headers ..
Note that the python2.7-config tool is useful for locating the library and include directory.

Relevant Citations
------------------

If this library helped you in a publication, please cite our [*Reflections on Simultaneous Impact*](http://www.cs.columbia.edu/cg/rosi/) paper so others can find our code and benefit as well.
For further details on the algorithms and models available in SCISim, see the following publications:

* [*Reflections on Simultaneous Impact*](http://www.cs.columbia.edu/cg/rosi/)  
[Breannan Smith](http://breannansmith.com), [Danny M. Kaufman](http://www.adobe.com/technology/people/seattle/danny-kaufman.html), [Etienne Vouga](http://www.cs.utexas.edu/users/evouga/index.html), [Rasmus Tamstorf](http://www.disneyresearch.com/people/rasmus-tamstorf/), [Eitan Grinspun](http://www.cs.columbia.edu/~eitan/)
  * Describes the Generalized Reflections and Generalized Restitution impact models.

* [*Staggered Projections for Frictional Contact in Multibody Systems*](http://www.cs.ubc.ca/labs/sensorimotor/projects/sp_sigasia08/)  
[Danny M. Kaufman](http://www.adobe.com/technology/people/seattle/danny-kaufman.html), [Shinjiro Sueda](http://www.calpoly.edu/~ssueda/), [Doug L. James](http://www.cs.cornell.edu/~djames/), [Dinesh K. Pai](http://www.cs.ubc.ca/~pai/)
  * Describes the globally coupled Staggered Projections friction solver.
