#!/bin/sh
CP_JAV=`rospack find tug_ist_diagnosis_repair`/java
CP_JAV_SOURCE=$CP_JAV/source
CP_JAV_CLASSES=$CP_JAV/classes
cd $CP_JAV_SOURCE
javac -classpath .:pddl4j.jar -d $CP_JAV_CLASSES/ Diagnosis_planner.java
echo *******JAVA FILES SUCCESSFULLY COMPILED!********

