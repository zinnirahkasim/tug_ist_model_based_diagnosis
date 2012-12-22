#!/bin/sh
CP_JAV=`rospack find diagnosis_repair`/java
CP_JAV_SOURCE=$CP_JAV/source
CP_JAV_CLASSES=$CP_JAV/classes
cd $CP_JAV_CLASSES
echo $1
java Diagnosis_planner $1
