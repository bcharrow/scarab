# makefile: host/aSteepGen/

SHELL=/bin/sh
PLATFORM=$(shell uname)
ARCH=`arch`


.PHONY : all
all : debug release


.PHONY : debug
debug :
	make -sf makefile.$(PLATFORM) ARCH=$(ARCH) debug


.PHONY : release
release :
	make -sf makefile.$(PLATFORM) ARCH=$(ARCH) release


.PHONY : clean
clean :
	make -sf makefile.$(PLATFORM) ARCH=$(ARCH) clean
	rm -rf unix_aSteepGen_Data
	rm -rf palmos_aSteepGen_Data
	rm -rf win32_aSteepGen_Data
	rm -rf wince_aSteepGen_Data
	rm -rf "macos_aSteepGen Data"
	rm -rf macx_aSteepGen_Data
	rm -f .DS_Store
	rm -rf .AppleFileInfo
	touch foo~
	rm -f *~
