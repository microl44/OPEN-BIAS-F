@echo off
cls
Set Sleep=0
:start
if %Sleep% == 1 ( goto end )
roslaunch ..\main.launch models:=6
echo Finished the test launch file
Set /A Sleep+=1
echo %Sleep%
goto start
:end
echo "FINISHED!!"