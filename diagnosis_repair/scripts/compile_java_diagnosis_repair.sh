#!/bin/sh
CP_JAV=`rospack find diagnosis_repair`/java
CP_JAV_SOURCE=$CP_JAV/source
CP_JAV_CLASSES=$CP_JAV/classes
echo $CP_JAV_SOURCE
cd $CP_JAV_SOURCE
javac -classpath .:pddl4j.jar -d $CP_JAV_CLASSES/ $CP_JAV_SOURCE/Diagnosis_planner.java
echo java diagnosis repair files have been compiled!

