. ./paths.sh
cd src
javac -cp $XUGGLER_HOME/share/java/jars/xuggle-xuggler.jar -Xlint:deprecation -d ../bin/ ./Ros/*.java ./Main/*.java ./Common/*.java ./LZ77/*.java ./AEC/*.java
cd ..
