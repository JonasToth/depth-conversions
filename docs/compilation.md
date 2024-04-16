# How to build the software

The project itself tests various `clang` and `gcc` toolchains, versions,
`libc++`, `stdlibc++`, `libc`, `musl` and optimization options to get better
coverage and find more bugs.
It utilizes the `clang` static analysis tools as well. It is expected to be
green with all of these.

Given this good coverage changes are, that you can compile the code on your
system with a modern toolchain. The project relies on **C++17** and does not
attempt to be backwards compatible.

## Don't

The project provide `docker` images with the binaries included. You can
download prebuilt binaries as well, they do not rely on any dynamic
dependencies except the system libraries. If you `libc` is new enough, they
should work out of the box!

This is not the case for `RedHat Enterprise 5` (or `centos 5`).

## But I need to build from source

You can try `./scripts/ci/normal_build.bash` for building, it will create the
directory `build` inside the root of you repository and build there.
This script is controlled via environment variables and fairly easy to
understand (if you are familiar with bash scripts).
It is the workhorse for the CI of this project.

The resulting binaries are in the directory `build/src/apps`.

## Minimum required versions

The following tools are required for building:

- `CMake >= 3.13` you can always install a recent `CMake` with
  [pip](https://pypi.org/project/cmake/)
- a modern C++ compiler with C++17 support (`gcc/g++ >= 7.x`; `clang >= 8`;
  theoretically `Visual Studio >= 2019`, but its not tested)
- the sytem linker works, but you can use `lld` for linking as well
- **Optional**: `ninja` as build-tool. `make` works as well, but is not as fast

Other dependencies are optional as there is always a fallback to bundled
software versions.

### OpenCV

This project relies on the non-free components from `OpenCV`. These are not
provided by for example ubuntu and might require a custom installation of
`OpenCV`.
The version should at least be `3.0`, if there are any comlications you can
just use the bundled version and link that statically.

### libEigen3

As `Eigen3` itself is a header-only C++-library it is not that complicated as
`OpenCV`. You can check if your system provides a system package, like ubuntu
with `libeigen3-dev` - version 3.3 is required.
You can fallback to the bundled version as well, it does not require
compilation.

### Boost

Because the project requires a very new `Boost` (see `dependencies.md`) the
version provided by your package manager might be outdated.
The project has custom building included. But you can of course build your
own boost and install it into your system.

## Build with everything included

This build takes the longest, produces the biggest executables but is most
portable. It will statically link the C++-standard-library, OpenCV and other
dependencies like libpng.

Fire your favorite shell up, get the code and do the following:

```bash
$ pwd
> Path to the project git/source
$ ls
> ... cmake .. src scripts ...

# Control you build tools, the linker is set with `-DCMAKE_LINKER`, see below
$ export CC=gcc-9
$ export CXX=g++-9

# Create a directory to build in, you can use a different name then 'build'
$ mkdir build && cd build
# The following command configures the build to statically link the standard
# library and use link-time-optimization (inter-procedural-optimization = ipo).
# Tests and benchmarks are build as well but not necessary. Still good to know
# if everything works!
#
# Note: `-G Ninja` to use 'ninja' later.
#       leaving this out will generate 'Makefiles'.
$ cmake .. \
   -G Ninja \
   -DCMAKE_BUILD_TYPE=Release \
   -DCMAKE_LINKER=ld \
   -DUSE_LIBCXX=OFF \
   -DBUILD_TESTING=ON \
   -DFORCE_BUNDLED_CV=ON \
   -DFORCE_BUNDLED_EIGEN=ON \
   -DWITH_TESTING=ON \
   -DWITH_CONTRACT_EXCEPTION=ON \
   -DWITH_STATIC_STDCXXLIB=ON \
   -DWITH_IPO=ON \
   -DWITH_BENCHMARK=OFF
# Build OpenCV first and download Eigen3. After that project is reconfigured
# to use these dependencies and resolve everything properly.
$ ninja dependencies # Optionally 'make -j$(nproc) dependencies'
> This will take a while, building OpenCV and others
$ cmake .
$ ninja # Optionally 'make -j$(nproc)'
> Building this repository
$ ctest -j$(nproc) --output-on-failure -R "test_" .
> Unit and integration tests should succeed
$ ctest -j1 -V -R "bm_" .
> Benchmarks should run one-by-one as they would influence each other.
> '-V' ensures everything is output.

# Check what the executables depend on
$ cd src/apps
$ ldd depth2x
> check that there is nothing like `libstdc++.so` and no libjpeg and so on
> required.
> OpenCV is known for just linking the dynamic version of dependencies.
> If you have `libjpeg` installed it might be linked dynamically.
>
> If you have a problem with that you could use a docker container for
> building.
> Alternativally opencv allows you to specify a path to these libraries where
> you can provide the static library.
> This is best done with googling and requires patching of
> `cmake/opencv_options.cmake` in this repository.
```

## Build with system libraries

This build will most likely only work on your own system and is therefore less
portable.
Because a relatively new toolchain is required its dynamic libraries are
required to be present on the system. The same goes with all other
dependencies.
For builds that just reside on the machine building or controlled environment
where everything can be installed as required there should be no problems.
These builds are faster as well (and therefore used for some of the CI jobs).

```bash
# Setup everything to build, very similar to the first part above.
$ mkdir build && cd build

# e.g. ubuntu-18.04 should just work like this, as the system toolchain is new
# enough.
# This build is a minimal build.
$ cmake .. \
   -DCMAKE_BUILD_TYPE=Release \
   -DBUILD_TESTING=OFF
> The output should note, that 'OpenCV' and 'Eigen3' are found. If not the
> bundled version are used again!

$ ninja
> Should start building the project. If 'nothing to do' happens, the
> dependencies were not resolved.
> In this case do 'ninja dependencies && cmake . && ninja'
```

## Miscelaneous

The build configuration supports enabling vector instructions for various
X86 extensions. For a build that just resides on your own machine we recommend
`WITH_MARCH_NATIVE=ON`, as the compiler will figure out the right options.

Other options are `WITH_SSE42`, `WITH_AVX`, `WITH_AVX2`.
By default `WITH_SSE42` is enabled.
