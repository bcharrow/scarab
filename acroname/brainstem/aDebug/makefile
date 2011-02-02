# host/aDebug/makefile

SHELL := /bin/sh


.PHONY : all
all : 


.PHONY : prep
prep :
	make -C aUnix prep


.PHONY : clean
clean : 
	make -C aAsset clean
	make -C aUnix clean
	make -C aSystem clean
	rm -rf .AppleFileInfo
	rm -rf .DS_Store
	touch foo~
	rm -f *~
