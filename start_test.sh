#!/bin/sh

xterm -T roscore -e roscore &

sleep 1

xterm -T "Sender on Topic1" -e rosrun diagnosis_observers Triggering.py &

sleep 1

xterm -T "Sender on Topic2" -e rosrun diagnosis_observers Triggered.py &

sleep 1

xterm -T "GObs 5Hz" -e rosrun diagnosis_observers GObs.py _topic:=/Topic1 _frq:=5 _dev:=1 _ws:=10 &

sleep 1

xterm -T "MObs time=500ms" -e rosrun diagnosis_observers MObs.py _in_topic:=Topic1 _out_topic:=Topic2 _tm:=500 &

sleep 1

xterm -T diagnosis_engine -e rosrun rosjava_bootstrap run.py diagnosis_engine diagnosis_engine __name:=engine &

sleep 3

xterm -T "Model Server" -e rosrun diagnosis_observers sd_node.py _model:="./diagnosis_observers/SD.yaml" &	

sleep 1

xterm -T diagnosis_repair -e rosrun rosjava_bootstrap run.py diagnosis_repair planner __name:=repair &

echo -n "Press [ENTER] to stop ...: "
read var_name

killall -9 xterm
