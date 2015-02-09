# damsonlib
The DAMSON lib runtime environment

Documentation on damson website: http://damson.sites.sheffield.ac.uk/

Author: Paul Richmond
Copyright: Univeristy of Sheffield
Contact: p.richmond@Sheffield.ac.uk

#ARM GNU tools compiler

The supported arm tools are available from the mentor graphics (formally code sorcery) website below. The supported (i.e. tested) gcc version is 4.5.2 (Sourcery G++ Lite 2011.03-41). Newer versions may work but are untested.

https://sourcery.mentor.com/GNUToolchain/release1803?lite=arm

In order to add the toolkit binaries directory to your bash profile path. Open your bash profile using the following command;
'''
sudo gedit ~/.bashrc
'''

The following should be appended to the end of the file where /home/paul/data/arm/gnutools/bin is the directory of the arm GNU tools binaries directory.

> \#add arm tools to path
> GCCARMPATH=/home/paul/arm/gnutools/bin
> PATH=$GCCARMPATH:$PATH:.

Close and reopen the terminal and test by calling the following which should give the GCC and code sourcery version;

> arm-none-linux-gnueabi-gcc -v

#Compiling DAMSONLIB

Compile using make

> make -f gnu.make

damsonlib.o will be created in the damsonlib directory.
