# makefile: brainstem/aJava/makefile

SHELL=/bin/sh

ARCH=`arch`

.PHONY : all
all :
	javac acroname/*.java
	jar cvf acroname.jar acroname/*.class
	javac -classpath acroname.jar garcia_triangle.java
	javac -classpath acroname.jar garcia_talk.java
	javac -classpath acroname.jar garcia_ir.java
	javac -classpath acroname.jar garcia_boom.java
	javac -classpath acroname.jar garcia_slalom.java

.PHONY : clean
clean :
	touch foo.class
	rm -f *.class
	touch acroname/foo.class
	rm -f acroname/*.class
	touch acroname.jar
	rm -f acroname.jar
	touch foo~
	rm -f *~
	rm -f .DS_Store
	rm -rf .AppleFileInfo
