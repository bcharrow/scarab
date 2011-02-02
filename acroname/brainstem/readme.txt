
       Linux Build and Installation Instructions

The BrainStem software is supplied in source-code form for 
Linux.  This allows the distribution to be built for 
particular platform/processor combinations specified by the 
user.

--------------------------------------------------------------
Building

The basic build process uses a recursive make.  The output
of all these underlying directories ends up in the two folders:

brainstem/aDebug/aUnix/<ARCH>

  and
  
brainstem/aRelease/aUnix/<ARCH>

Where <ARCH> is the processor type you have specified.  If 
you don't specify a processor type, the build will default to
the native type where the compiler is running.  Currently the
following architectures are automatically supported:

  ARM
  i386
  i586
  i686

--------------------------------------------------------------
Cross-Compiling

To specify an architecture other than your compiler's native
architecture, you specify the architecture with a make define.
For instance, if my native compiler is in my default path, I
would do the following to get a cross-compiled version for the
ARM processor:

1. Change the path to ensure the ARM compiler is found first:

  % PATH=/usr/local/arm/bin:$PATH

2. Initiate the build with the proper define for ARM as well
   as the specific compilers that should be used:

  % make CC=arm-linux-gcc CCP=arm-linux-g++ ARCH=ARM

--------------------------------------------------------------
Installing

When all is built, you can build a tarball stripped of all 
sources for installation at /usr/local/acroname by typing:

  % make tarball

If you are cross-compiling for another platform like ARM,
you would type:

  % make ARCH=ARM tarball

To install this, you copy the resulting dist_ARM.tgz to
the root directory on your destination machine and then
expand th archive using:

  % tar -xzvmf dist_ARM.tgz

You may also need to include the c++ libraries from your 
build environement, depending on the distribution files
in your embedded ARM environment.

Some of the acroname code also looks up the hostname in
/etc/hosts to get the IP address of the machine.  You may
need an entry there to allow the code to find the IP 
address using this method.

Finally, many Acroname programs use a config file to set
parameters like the serial port being used, baudrate, etc.
You may need to make or modify these in the aBinary 
directory to get the settings you would like.

--------------------------------------------------------------
Running

Since the output of the entire make process deposits both
the executables and the shared libraries into one output 
directory, there is a small shell script "run" that sets 
the LD_LIBRARY_PATH environment variable so the executables
can find the shared libraries when run from this output
directory.  For example, to run the executable "aGarciaApp"
in the output directory, you would type:

  % ./run aGarciaApp

If you like, you can move the shared libraries to a default
library location on your machine to allow these executables
to be run from anywhere.

A good test of your build is to run the aIOTests application
from the aDebug/aUnix/<ARCH> and aDebug/aUnix/<aARCH> 
directories.  This simple program hammers the basic aIO code
and will stall for a short while at the end when doing some 
timing tests.  If this the build completes successfully and 
your aIOTests seem to validate (on your target platform), you
likely have a clean build.

Notes:

Serial Port permissions:

 Many distributions of Linux prohibit reading writing to
 serial ports by default.  The BrainStem code assumes these
 are both readable and writeable.  As root, you may need to
 adjust these permissions for world read/write.

Endless Looping Builds:

 In some cases, the dates get out of whack in the distribution
 process.  If your build seems to loop endlessly, this may 
 have happened.  Try unwrapping the archive with the -m option
 to tar which doesn't preserve dates.  Things should work after
 this.

Stargate Notes:

 We use the GCC 3.3 build described on the platformx site:

  http://platformx.sourceforge.net/

 for the builds of this tree for the Stargate.  We have also
 tried GCC 3.4 but the library support isn't built in on the
 Stargate standard distribution build.
