The Structured Contact Impact Simulator
=======================================

Welcome to **SCISim** (pronounced skiz-em), the Structured Contact Impact Simulator! **SCISim** is a C++14 software package designed to simulate systems of rigid bodies where the evolution is dominated by collisions. **SCISim** respects essential symmetries and structures from the continuous setting at coarse temporal discretizations through the use of geometric integrators for the unconstrained and constrained dynamics. In practice, this leads to stable and predictable behavior across timesteps. For a preview of the types of simulations that **SCISim** can perform, please see our demo reel (coming soon!).


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

* [Qt4](http://qt.digia.com/): A user interface library to provide graphical front ends. Available through most standard package managers.

* [HDF5](https://www.hdfgroup.org/HDF5/): A binary file format for configuration and force output. Available through most standard package managers.

* [Ipopt](https://projects.coin-or.org/Ipopt): A nonlinear optimization package for global LCP solves, for the Staggered Projections friction solver, and for the Generalized Reflections and Generalized Restitution impact models. Ipopt requires a Fortran compiler.

* [HSL2013](http://www.hsl.rl.ac.uk/ipopt/): A collection of sparse linear solvers suggested for use with Ipopt.


Optional Dependencies
---------------------

SCISim optionally supports other numerical solvers:

* [QL](http://www.ai7.uni-bayreuth.de/ql.htm): A quadratic program solver useful for small to medium problems. To use QL, place the file ql.for under SCISim/Math/QL and rebuild. QL requires a Fortran compiler, and is available for academic use.


Quickstart Guide
----------------

To obtain a minimal demo build that simulates colliding triangle meshes:

1. Install Qt4 and CMake. These packages are available from most standard package managers.

2. Clone this repository and change into the project root:

        git clone git@github.com:breannansmith/scisim.git
        cd scisim

3. From the project root, run the script get_dependencies.sh to download, extract, and verify the required dependencies:

        ./get_dependencies.sh

4. Create a build directory under the project root and change into this directory:

        mkdir build
        cd build

5. Run CMake to create the build system with Qt4 and HDF5 enabled:

        cmake -DUSE_QT4=ON -DUSE_HDF5=ON ..

6. Build SCISim:

        make

7. Load the example simulation:

        cd rigidbody3dqt4
        ./rigidbody3d_qt4 assets/Meshes/dragon_drop/two_dragon_drop.xml

8. Press space to run the simulation!


Building SCISim
---------------

SCISim uses the CMake build system. SCISim is tested regularly against recent versions of the GCC, LLVM, and Intel compiler toolchains on both Linux and OS X. A minimal SCISim build requires only a C and C++ compiler. The optional QL and Ipopt dependencies require an additional Fortran compiler.

As an example, to build SCISim with an optional Fortran compiler using the Intel toolchain, from a build directory, run:

    CC=icc CXX=icpc FC=ifort cmake ..

Substitute out the compiler names as required by your platform. The build system can be configured via the command line by running

    ccmake ..

from the build directory.

Options of note include:

* CMAKE_BUILD_TYPE: General build type that enables various optimization and compiler flags. Options are: Debug, Release, RelWithDebInfo, MinSizeRel

* STRICT_BUILD: Enables agressive warnings and treats warnings as errors. Recommended for development.

* USE_HDF5: Enables state and force output via [HDF5](https://www.hdfgroup.org/HDF5/) files.

* USE_IPOPT: Enables the [Ipopt](https://projects.coin-or.org/Ipopt) nonlinear solver.

* USE_OPENMP: Enables OpenMP support. Highly recommend for the [So-bogus](https://bitbucket.org/gdaviet/so-bogus) solver.

* USE_QT4: Enables support for graphical front ends using [Qt4](http://qt.digia.com/).

* USE_PYTHON: Enables support for embedded Python language scripting. Required for kinematic scripting.


Platform Specific Issues
------------------------

* OS X
  * There are performance regressions with GCC and So-bogus when building with the GCC toolchain provided by MacPorts.
  * There are a number of strange behaviors with the Qt4 frontend when building against the version from MacPorts. I intend to upgrade to Qt5, which should remedy these issues.
  * CMake will often pull in different versions of the Python interpreter and the Python library, requiring the user to explicitly pass the location of the Python library to CMake. For example, to build with MacPorts' Python 2.7:

            cmake -DPYTHON_LIBRARY=/opt/local/lib/libpython2.7.dylib -DPYTHON_INCLUDE_DIR=/opt/local/Library/Frameworks/Python.framework/Headers ..


Relevant Citations
------------------

If this library helped you in a publication, please cite our [*Reflections on Simultaneous Impact*](http://www.cs.columbia.edu/cg/rosi/) paper so others can find our code and benefit as well.
For further details on the algorithms and models available in SCISim, see the following publications:

* [*Reflections on Simultaneous Impact*](http://www.cs.columbia.edu/cg/rosi/)  
[Breannan Smith](http://breannansmith.com), [Danny M. Kaufman](http://www.adobe.com/technology/people/seattle/danny-kaufman.html), [Etienne Vouga](http://www.cs.utexas.edu/users/evouga/index.html), [Rasmus Tamstorf](http://www.disneyresearch.com/people/rasmus-tamstorf/), [Eitan Grinpsun](http://www.cs.columbia.edu/~eitan/)
  * Describes the Generalized Reflections and Generalized Restitution impact models.

* [*Staggered Projections for Frictional Contact in Multibody Systems*](http://www.cs.ubc.ca/labs/sensorimotor/projects/sp_sigasia08/)  
[Danny M. Kaufman](http://www.adobe.com/technology/people/seattle/danny-kaufman.html), [Shinjiro Sueda](http://www.calpoly.edu/~ssueda/), [Doug L. James](http://www.cs.cornell.edu/~djames/), [Dinesh K. Pai](http://www.cs.ubc.ca/~pai/)
  * Describes the globally coupled Staggered Projections friction solver.


Building Ipopt
--------------

1. Create a directory in which to install Ipopt and export the path. Replace YOUR_INSTALL_DIR with the actual desired installation directory.

        export IPOPT_DIR=YOUR_INSTALL_DIR

2. Download Ipopt

        wget http://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.4.tgz

3. Extract Ipopt

        tar -xvzf Ipopt-3.12.4.tgz

4. Download Metis

        ./ThirdParty/Metis/get.Metis

5. Download the HSL Solvers
    1. Complete the registration form at http://www.hsl.rl.ac.uk/ipopt/ for HSL2013
    2. When you receive an email with a download link, extract the archive coinhsl-2014.01.10.tar.gz to ThirdParty/HSL/coinhsl

6. Create and change into a build directory

7. Configure Ipopt (see examples below)

8. Execute make

9. Execute make test

10. Check for errors

11. Execute make install

(Note that if you are building with GCC, but have the Intel compilers in your path via /opt/intel/iccvars.sh, the Ipopt settings get polluted with Intel includes, for some reaosn.)

Some example configurations for Ipopt include:

* MacPort GCC toolchain, OS X Accelerate framework, single threaded

        ../configure CXX=g++-mp-5 CC=gcc-mp-5 F77=gfortran-mp-5 --with-blas="-framework accelerate" --prefix=$IPOPT_DIR

* Intel compiler toolchain, MKL BLAS, multithreaded

        ../configure CXX=icpc CC=icc F77=ifort ADD_CFLAGS=-openmp ADD_FFLAGS=-openmp ADD_CXXFLAGS=-openmp --with-blas="-L$MKLROOT/lib/intel64 -lmkl_intel_lp64 -lmkl_sequential -lmkl_core" --prefix=$IPOPT_DIR

* The Intel compiler toolchain with MKL BLAS and MKL Pardiso. Note that Pardiso support in Ipopt is still experimental.
    
        ../configure CXX=icpc CC=icc F77=ifort ADD_CXXFLAGS="-DHAVE_PARDISO_MKL -DHAVE_PARDISO_PARALLEL" --with-blas="-L${MKLROOT}/lib -lmkl_intel_lp64 -lmkl_core -lmkl_intel_thread -lpthread -lm" --with-pardiso="-L${MKLROOT}/lib -lmkl_intel_lp64 -lmkl_core -lmkl_intel_thread -lpthread -lm" --prefix=$IPOPT_DIR

* The GCC compiler toolchain and MKL BLAS

        ../configure CXX=g++ CC=gcc F77=gfortran ADD_CFLAGS=-fopenmp ADD_FFLAGS=-fopenmp ADD_CXXFLAGS=-fopenmp --with-blas="-L$MKLROOT/lib/intel64 -lmkl_intel_lp64 -lmkl_sequential -lmkl_core -lm" --prefix=$IPOPT_DIR

* To build with debug support, add the following to any of the above commands

        --enable-debug -with-ipopt-checklevel=1
