# makefile: host/aExamples/makefile

SHELL := /bin/sh
ARCH := `arch`


.PHONY : all
all : debug release


.PHONY : debug
debug : 
	make -C aStub ARCH=$(ARCH) debug
	make -C aSimpleRange ARCH=$(ARCH) debug
	make -C aDataLogger ARCH=$(ARCH) debug
	make -C aSimpleHTTP ARCH=$(ARCH) debug
	make -C aPPRKStub ARCH=$(ARCH) debug


.PHONY : release
release : 
	make -C aStub ARCH=$(ARCH) release
	make -C aSimpleRange ARCH=$(ARCH) release
	make -C aDataLogger ARCH=$(ARCH) release
	make -C aSimpleHTTP ARCH=$(ARCH) release
	make -C aPPRKStub ARCH=$(ARCH) release


.PHONY : clean
clean : 
	make -C aStub ARCH=$(ARCH) clean
	make -C aSimpleRange ARCH=$(ARCH) clean
	make -C aDataLogger ARCH=$(ARCH) clean
	make -C aSimpleHTTP ARCH=$(ARCH) clean
	make -C aPPRKStub ARCH=$(ARCH) clean
	touch foo~
	rm -f *~
	rm -rf .AppleFileInfo
	rm -f .DS_Store
