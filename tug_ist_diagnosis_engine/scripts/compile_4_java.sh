#!/bin/sh
CP_JAV=`rospack find diagnosis_engine`/java
CP_JAV_SOURCE=$CP_JAV/source/
CP_JAV_CLASSES=$CP_JAV/classes/
javac -classpath $CP_JAV_SOURCE -d $CP_JAV_CLASSES $CP_JAV_SOURCE/ATPInterface/*.java
echo *******JAVA FILES SUCCESSFULLY COMPILED!********

