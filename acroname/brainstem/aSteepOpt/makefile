# makefile: host/aSteepOpt/

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
	make -sf makefile.$(PLATFORM) clean
	rm -rf unix_aSteepOpt_Data
	rm -rf palmos_aSteepOpt_Data
	rm -rf win32_aSteepOpt_Data
	rm -rf wince_aSteepOpt_Data
	rm -rf "macos_aSteepOpt Data"
	rm -rf macx_aSteepOpt_Data
	rm -f .DS_Store
	rm -rf .AppleFileInfo
	touch foo~
	rm -f *~
