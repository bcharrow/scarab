# makefile: host/aRobot/makefile

SHELL=/bin/sh
PLATFORM=$(shell uname)
ARCH=`arch`

.PHONY : all
all : debug release

.PHONY : debug
debug :
	make -f makelib.$(PLATFORM) ARCH=$(ARCH) debug
	make -f makepkg.$(PLATFORM) ARCH=$(ARCH) debug
	make -f maketest.$(PLATFORM) ARCH=$(ARCH) debug
	make -f makeapp.$(PLATFORM) ARCH=$(ARCH) debug
	make -f makeagent.$(PLATFORM) ARCH=$(ARCH) debug
	make -f makeshell.$(PLATFORM) ARCH=$(ARCH) debug

.PHONY : release
release :
	make -f makelib.$(PLATFORM) ARCH=$(ARCH) release
	make -f makepkg.$(PLATFORM) ARCH=$(ARCH) release
	make -f maketest.$(PLATFORM) ARCH=$(ARCH) release
	make -f makeapp.$(PLATFORM) ARCH=$(ARCH) release
	make -f makeagent.$(PLATFORM) ARCH=$(ARCH) release
	make -f makeshell.$(PLATFORM) ARCH=$(ARCH) release

.PHONY : clean
clean :
	make -C aCommon ARCH=$(ARCH) clean
	make -C aRobot ARCH=$(ARCH) clean
	make -C aRobotApp ARCH=$(ARCH) clean
	make -C aRobotTest ARCH=$(ARCH) clean
	make -C aRobotPackager ARCH=$(ARCH) clean
	make -f makelib.$(PLATFORM) ARCH=$(ARCH) clean
	make -f makepkg.$(PLATFORM) ARCH=$(ARCH) clean
	make -f maketest.$(PLATFORM) ARCH=$(ARCH) clean
	make -f makeapp.$(PLATFORM) ARCH=$(ARCH) clean
	make -f makeagent.$(PLATFORM) ARCH=$(ARCH) clean
	rm -rf .AppleFileInfo
	rm -rf .DS_Store
	touch foo~
	rm -f *~
