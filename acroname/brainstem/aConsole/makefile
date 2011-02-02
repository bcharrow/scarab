# makefile: host/aConsole/

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
	make -C aCommon ARCH=$(ARCH) clean
	make -C aUnix ARCH=$(ARCH) clean
	make -f makefile.$(PLATFORM) ARCH=$(ARCH) clean
	rm -rf .AppleFileInfo
	rm -rf .DS_Store
	touch foo~
	rm -f *~
