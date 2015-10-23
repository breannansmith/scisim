Building Ipopt (Optional)
=========================

1. Create a directory in which to install Ipopt and export the path. Replace YOUR_INSTALL_DIR with the actual desired installation directory:

        export IPOPT_DIR=YOUR_INSTALL_DIR

2. Download Ipopt:

        wget http://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.4.tgz

3. Extract Ipopt:

        tar -xf Ipopt-3.12.4.tgz

4. Download Metis:

        ./Ipopt-3.12.4/ThirdParty/Metis/get.Metis

5. Download the HSL Solvers.
    1. Complete the registration form at http://www.hsl.rl.ac.uk/ipopt/ for HSL2013
    2. When you receive an email with a download link, extract the archive coinhsl-2014.01.10.tar.gz to Ipopt-3.12.4/ThirdParty/HSL/coinhsl

6. Create and change into a build directory:

        mkdir Ipopt-3.12.4/build
        cd Ipopt-3.12.4/build

7. Configure Ipopt (see examples below)

8. Build Ipopt:

        make

9. Test Ipopt:

        make test

10. Check for errors

11. Install Ipopt:

        make install

(Note that if you are building with GCC, but have the Intel compilers in your path via /opt/intel/iccvars.sh, the Ipopt settings get polluted with Intel includes, for some reason.)

Some example configurations for Ipopt include

* Homebrew GCC toolchain, OS X Accelerate framework, single threaded

        ../configure CXX=g++-5 CC=gcc-5 F77=gfortran-5 --with-blas="-framework accelerate" --prefix=$IPOPT_DIR

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
