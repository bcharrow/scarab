# host/aDebug/aUnix/makefile

SHELL=/bin/sh
PLATFORM=$(shell uname)
ARCH=`arch`


.PHONY : all
all :


.PHONY : prep
prep :
	make -f makefile.$(PLATFORM) ARCH=$(ARCH) prep


.PHONY: clean
clean :
	make -f makefile.$(PLATFORM) ARCH=$(ARCH) clean
	make -C i386 clean
	make -C i586 clean
	make -C i686 clean
	make -C x86_64 clean
	make -C ARM clean
	rm -rf .AppleFileInfo
	rm -f .DS_Store
	touch foo~
	rm -f *~


.PHONY: tests
tests :
	make -f makefile.$(PLATFORM) tests


.PHONY: output
output :
	make -C i386 output
	make -C i586 output
	make -C i686 output
	make -C x86_64 output
	make -C ARM output
