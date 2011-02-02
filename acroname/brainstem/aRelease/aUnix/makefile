# host/aRelease/aUnix/makefile

SHELL=/bin/sh
PLATFORM=$(shell uname)
ARCH=`arch`


.PHONY : all
all : 


.PHONY : prep
prep :
	make -f makefile.$(PLATFORM) ARCH=$(ARCH) prep


.PHONY : clean
clean : 
	make -f makefile.$(PLATFORM) ARCH=$(ARCH) clean
	make -C i386 clean
	make -C i586 clean
	make -C i686 clean
	make -C x86_64 clean
	make -C ARM clean
	rm -rf .AppleFileInfo
	rm -rf .DS_Store
	touch foo~
	rm -f *~


.PHONY: output
output :
	make -f makefile.$(PLATFORM) ARCH=$(ARCH) output
