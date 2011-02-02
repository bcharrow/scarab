# makefile: host/aExamples/aSimpleHTTP/makefile

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


.PHONY : tests
tests :
	make -f make_tests.$(PLATFORM) ARCH=$(ARCH) tests


.PHONY : clean
clean :
	make -f makefile.$(PLATFORM) ARCH=$(ARCH) clean
	touch foo~
	rm -f *~
	rm -rf .AppleFileInfo
	rm -rf .DS_Store
