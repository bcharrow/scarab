# makefile: host/aStem/

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
	make -f makefile.$(PLATFORM) ARCH=$(ARCH) clean
	make -C aCommon ARCH=$(ARCH) clean
	rm -f .DS_Store
	rm -rf .AppleFileInfo
	touch foo~
	rm -f *~
