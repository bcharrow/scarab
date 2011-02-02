# makefile: host/aTEAvm/makefile

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
	rm -rf unix_aTEAvm_Data
	rm -rf palmos_aTEAvm_Data
	rm -rf win32_aTEAvm_Data
	rm -rf wince_aTEAvm_Data
	rm -rf "macos_aTEAvm Data"
	rm -rf macx_aTEAvm_Data
	rm -f .DS_Store
	rm -rf .AppleFileInfo
	touch foo~
	rm -f *~
