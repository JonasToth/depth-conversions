[![pipeline status](https://git.informatik.tu-freiberg.de/jtoth/master-thesis/badges/master/pipeline.svg?style=flat-square)](https://git.informatik.tu-freiberg.de/jtoth/master-thesis/commits/master)
[![coverage report](https://git.informatik.tu-freiberg.de/jtoth/master-thesis/badges/master/coverage.svg?style=flat-square)](https://git.informatik.tu-freiberg.de/jtoth/master-thesis/commits/master)
[![Standard](docs/cxx17.svg)](https://en.wikipedia.org/wiki/C%2B%2B#Standardization)
[![Download](docs/download.svg)](https://git.informatik.tu-freiberg.de/jtoth/master-thesis/-/archive/master/master-thesis-master.tar.gz)

# master-thesis

Repository containing all code and stuff for my master thesis.
For more specific documentation check the `docs/` directory.

## What is this

Goal of this project is to localize depth image in a known 3D model.
This model can either be a laserscan, a reconstructed mesh or a dense
pointcloud.

All the science for that will be written in my master thesis.

## Getting the software

You can download the source code and compile it on your own system. This allows
for the most customization and most control over the final executable. This
project tries to support many compilers and environment but is very Linux
oriented. For more information on building see the
[Compilation Reference](docs/compilation.md).

It is possible to download artifacts from the CI setup. See the
[Pipelines Page](https://git.informatik.tu-freiberg.de/jtoth/master-thesis/pipelines?scope=branches&page=1)
that provides a download link for the artifacts. If there is no artifact that
works on your system you need to fall back to compiling on your own.

Please note, that artifacts from the `platform` stage require `libjpeg-dev` and
`libpng-dev` libraries as they are dynamically linked.
On Linux system you can check `ldd <binary-executable>` if all dynamic
dependencies are resolved. If yes, the binary should work, given the `libc` is
new enough ;)

Using a `docker` container with `ubuntu:18.04` should definitly work! More
robust ways to distribute, e.g. packages, are implemented when necessary and
time allows it.

## Contributing

Right now, not so much. As this is part of my master thesis I need to develop
it on my own. Bug Reports are of course always welcome!
