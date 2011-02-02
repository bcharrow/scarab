# makefile: host/aPPRK/

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
	rm -rf palmos_aIO_Data
	rm -rf win32_aIO_Data
	rm -rf wince_aIO_Data
	rm -rf "macos_aIO Data"
	rm -rf macx_aIO_Data
	rm -f .DS_Store
	touch foo~
	rm -f *~
