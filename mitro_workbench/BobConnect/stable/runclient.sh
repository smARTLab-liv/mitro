echo $1 $2 $3
. ./paths.sh
cd bin
f=`dirname $0`
java -Djava.library.path=$XUGGLER_HOME/lib \
	-cp \
$JDK_HOME/jre/lib/alt-rt.jar:\
$JDK_HOME/jre/lib/management-agent.jar:\
$JDK_HOME/jre/lib/javaws.jar:\
$JDK_HOME/jre/lib/jsse.jar:\
$JDK_HOME/jre/lib/alt-string.jar:\
$JDK_HOME/jre/lib/plugin.jar:\
$JDK_HOME/jre/lib/resources.jar:\
$JDK_HOME/jre/lib/deploy.jar:\
$JDK_HOME/jre/lib/jce.jar:\
$JDK_HOME/jre/lib/rt.jar:\
$JDK_HOME/jre/lib/charsets.jar:\
$JDK_HOME/jre/lib/ext/localedata.jar:\
$JDK_HOME/jre/lib/ext/dnsns.jar:\
$JDK_HOME/jre/lib/ext/mp3plugin.jar:\
$JDK_HOME/jre/lib/ext/sunpkcs11.jar:\
$JDK_HOME/jre/lib/ext/sunjce_provider.jar:\
$JDK_HOME/jre/lib/ext/customizer.jar:\
$JDK_HOME/jre/lib/ext/multiplayer.jar:\
$f:\
$XUGGLER_HOME/share/java/jars/xuggle-xuggler-test.jar:\
$XUGGLER_HOME/share/java/jars/logback-core.jar:\
$XUGGLER_HOME/share/java/jars/commons-cli.jar:\
$XUGGLER_HOME/share/java/jars/slf4j-api.jar:\
$XUGGLER_HOME/share/java/jars/logback-classic.jar:\
$XUGGLER_HOME/share/java/jars/xuggle-xuggler.jar \
Main.BobConnectClient $1 $2 $3

