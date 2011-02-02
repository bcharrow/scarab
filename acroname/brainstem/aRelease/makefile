# host/aRelease/makefile

SHELL := /bin/sh


.PHONY : all
all :


.PHONY : prep
prep :
	make -C aUnix prep


.PHONY : clean
clean : 
	make -C aUnix clean
	make -C aSystem clean
	rm -rf .AppleFileInfo
	rm -f .DS_Store
	touch foo~
	rm -f *~
