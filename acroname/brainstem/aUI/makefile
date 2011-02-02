# makefile: host/aUI/

SHELL=/bin/sh
PLATFORM=$(shell uname)
ARCH=`arch`

all : debug release

debug :
	make -sf makefile.$(PLATFORM) ARCH=$(ARCH) debug

release :
	make -sf makefile.$(PLATFORM) ARCH=$(ARCH) release

.PHONY : clean
clean :
	make -sf makefile.$(PLATFORM) ARCH=$(ARCH) clean
	rm -rf palmos_aUI_Data
	rm -rf win32_aUI_Data
	rm -rf wince_aUI_Data
	rm -rf "macos_aUI Data"
	rm -rf macx_aUI_Data
	rm -rf .AppleFileInfo
	rm -f .DS_Store
	touch foo~
	rm -f *~

