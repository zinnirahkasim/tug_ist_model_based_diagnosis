#!/bin/sh
CP=`rospack find diagnosis_repair`/java
cd $CP
javac -classpath .:pddl4j.jar -d $CP/ $CP/Diagnosis_planner.java
echo java files have been compiled!

