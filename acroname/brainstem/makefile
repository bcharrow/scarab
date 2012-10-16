# makefile: host

SHELL=/bin/sh

ARCH=`arch`

all : prep debug release tests


# for Unix only
TARBALLDEST=dist/home/acroname/
.PHONY : tarball
tarball :
	mkdir -p $(TARBALLDEST)
	tar -C aRelease -cf - aAsset | tar -C $(TARBALLDEST) -xf -
	tar -C aRelease -cf - aSystem | tar -C $(TARBALLDEST) -xf -
	tar -C aRelease -cf - aObject | tar -C $(TARBALLDEST) -xf -
	tar -C aRelease -cf - aTest | tar -C $(TARBALLDEST) -xf -
	tar -C aRelease -cf - aUser | tar -C $(TARBALLDEST) -xf -
	mkdir $(TARBALLDEST)aPlugin
	tar -C aRelease/aUnix -cf - $(ARCH) | \
		tar -C $(TARBALLDEST) -xf -
	mv $(TARBALLDEST)$(ARCH) $(TARBALLDEST)aBinary
	chmod 555 $(TARBALLDEST)aBinary/*
	chmod 444 $(TARBALLDEST)aBinary/lib*.so
	rm -f $(TARBALLDEST)aAsset/makefile
	rm -f $(TARBALLDEST)aSystem/makefile
	rm -f $(TARBALLDEST)aBinary/makefile
	tar -C dist -czf dist_$(ARCH).tgz home
	rm -rf dist

prep :
	make -C aDebug ARCH=$(ARCH) prep
	make -C aRelease ARCH=$(ARCH) prep


debug :
	make -C aIO ARCH=$(ARCH) debug
# 	make -C aUI ARCH=$(ARCH) debug
	make -C aStem ARCH=$(ARCH) debug
# 	make -C aTEAvm ARCH=$(ARCH) debug
# 	make -C aLeaf ARCH=$(ARCH) debug
# 	make -C aSteepOpt ARCH=$(ARCH) debug
# 	make -C aSteepGen ARCH=$(ARCH) debug
# 	make -C aSteep ARCH=$(ARCH) debug
# 	make -C aPPRK ARCH=$(ARCH) debug
# 	make -C aGarcia ARCH=$(ARCH) debug
# 	make -C aRobot ARCH=$(ARCH) debug
# ifeq ($(ARCH),ARM)
# else
# 	make -C aConsole ARCH=$(ARCH) debug
# endif
# 	make -C aExamples ARCH=$(ARCH) debug


release : 
	make -C aIO ARCH=$(ARCH) release
	# make -C aUI ARCH=$(ARCH) release
	make -C aStem ARCH=$(ARCH) release
	# make -C aTEAvm ARCH=$(ARCH) release
	# make -C aLeaf ARCH=$(ARCH) release
	# make -C aSteepOpt ARCH=$(ARCH) release
	# make -C aSteepGen ARCH=$(ARCH) release
	# make -C aSteep ARCH=$(ARCH) release
	# make -C aPPRK ARCH=$(ARCH) release
	# make -C aGarcia ARCH=$(ARCH) release
	# make -C aRobot ARCH=$(ARCH) release
# ifeq ($(ARCH),ARM)
# else
# 	make -C aConsole ARCH=$(ARCH) release
# endif
# 	make -C aExamples ARCH=$(ARCH) release


tests :
	# make -C aIO ARCH=$(ARCH) tests


clean :
	make -C aIO ARCH=$(ARCH) clean
	make -C aUI ARCH=$(ARCH) clean
	make -C aStem ARCH=$(ARCH) clean
	make -C aTEAvm ARCH=$(ARCH) clean
	make -C aLeaf ARCH=$(ARCH) clean
	make -C aSteepOpt ARCH=$(ARCH) clean
	make -C aSteepGen ARCH=$(ARCH) clean
	make -C aSteep ARCH=$(ARCH) clean
	make -C aPPRK ARCH=$(ARCH) clean
	make -C aConsole ARCH=$(ARCH) clean
	make -C aGarcia ARCH=$(ARCH) clean
	make -C aRobot ARCH=$(ARCH) clean
	make -C aExamples ARCH=$(ARCH) clean
	make -C aDebug ARCH=$(ARCH) clean
	make -C aRelease ARCH=$(ARCH) clean
	rm -f .DS_Store
	rm -rf .AppleFileInfo
	touch foo~
	rm -f *~
