# makefile: host/aSteep/makefile

SHELL=/bin/sh
PLATFORM=$(shell uname)
ARCH=`arch`


.PHONY : all
all : debug release


.PHONY : debug
debug :
	make -f makefile.$(PLATFORM) ARCH=$(ARCH) debug


.PHONY : release
release :
	make -f makefile.$(PLATFORM) ARCH=$(ARCH) release


.PHONY : clean
clean :
	make -C aCommon
	make -C aUnix
	make -sf makefile.$(PLATFORM) ARCH=$(ARCH) clean
	rm -rf unix_aSteep_Data
	rm -f .DS_Store
	rm -rf .AppleFileInfo
	touch foo~
	rm -f *~
