#!/bin/sh
CP_JAV=`rospack find diagnosis_engine`/java
CP_JAV_CLASSES=$CP_JAV/classes/
java -classpath $CP_JAV_CLASSES ATPInterface.Server $1 $2
