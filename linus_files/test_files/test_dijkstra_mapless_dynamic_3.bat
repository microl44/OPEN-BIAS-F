@echo off
cls
Set Sleep=0
:start
if %Sleep% == 6 ( goto end )
roslaunch ..\dijkstra_mapless.launch models:=3
echo Finished the test launch file
Set /A Sleep+=1
echo %Sleep%
goto start
:end
echo "FINISHED!!"