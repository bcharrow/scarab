# makefile: host/aGarcia/makefile

SHELL=/bin/sh
PLATFORM=$(shell uname)
ARCH=`arch`


.PHONY : all
all : debug release


.PHONY : debug
debug :
	make -f makelib.$(PLATFORM) ARCH=$(ARCH) debug
	make -f makeapp.$(PLATFORM) ARCH=$(ARCH) debug
	make -f maketest.$(PLATFORM) ARCH=$(ARCH) debug
	make -f maketool.$(PLATFORM) ARCH=$(ARCH) debug
	make -f makejoy.$(PLATFORM) ARCH=$(ARCH) debug
	make -f makescript.$(PLATFORM) ARCH=$(ARCH) debug
	make -f makeagent.$(PLATFORM) ARCH=$(ARCH) debug


.PHONY : release
release :
	make -f makelib.$(PLATFORM) ARCH=$(ARCH) release
	make -f makeapp.$(PLATFORM) ARCH=$(ARCH) release
	make -f maketest.$(PLATFORM) ARCH=$(ARCH) release
	make -f maketool.$(PLATFORM) ARCH=$(ARCH) release
	make -f makejoy.$(PLATFORM) ARCH=$(ARCH) release
	make -f makescript.$(PLATFORM) ARCH=$(ARCH) release
	make -f makeagent.$(PLATFORM) ARCH=$(ARCH) release


.PHONY : clean
clean :
	make -C aCommon ARCH=$(ARCH) clean
	make -C aGarcia ARCH=$(ARCH) clean
	make -C aGarciaApp ARCH=$(ARCH) clean
	make -C aGarciaTest ARCH=$(ARCH) clean
	make -C aGarciaTool ARCH=$(ARCH) clean
	make -C aGarciaScript ARCH=$(ARCH) clean
	make -C aGarciaAgent ARCH=$(ARCH) clean
	make -C aSource ARCH=$(ARCH) clean
	make -C aUnix ARCH=$(ARCH) clean
	make -f makeapp.$(PLATFORM) ARCH=$(ARCH) clean
	make -f makelib.$(PLATFORM) ARCH=$(ARCH) clean
	make -f maketest.$(PLATFORM) ARCH=$(ARCH) clean
	make -f maketool.$(PLATFORM) ARCH=$(ARCH) clean
	make -f makescript.$(PLATFORM) ARCH=$(ARCH) clean
	make -f makeagent.$(PLATFORM) ARCH=$(ARCH) clean
	touch unix_Foo_Data
	rm -rf unix_*_Data
	rm -rf .AppleFileInfo
	rm -rf .DS_Store
	touch foo~
	rm -f *~
