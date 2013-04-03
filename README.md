sot-oscar
==========

This sofware provides some additional features to the sot-dyninv package
to deal with walking on non-planar surfaces. It is absolutely dependent
of sot-dyninv and can be considered just an extention of the latter.


Setup
-----

To compile this package, it is recommended to create a separate build
directory:

    mkdir build
    cd build
    cmake [OPTIONS] ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.


### Dependencies

The sot-oscar package depends on several packages which
have to be available on your machine.

 - Libraries:
   - sot-dyninv
   - sot-core (>= 1.0.0)
   - soth (>= 0.0.1)
 - System tools:
   - CMake (>=2.6)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)


[sot-core]: http://github.com/jrl-umi3128/sot-core
[soth]: ssh://softs.laas.fr/git/jrl/frameworks/soth
