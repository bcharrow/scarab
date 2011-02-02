# makefile: host/aLeaf/makefile

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
	rm -rf unix_aLeaf_Data
	rm -rf palmos_aLeaf_Data
	rm -rf win32_aLeaf_Data
	rm -rf wince_aLeaf_Data
	rm -rf "macos_aLeaf Data"
	rm -rf macx_aLeaf_Data
	rm -rf .DS_Store
