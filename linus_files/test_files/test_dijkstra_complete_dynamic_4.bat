@echo off
cls
Set Sleep=0
:start
if %Sleep% == 1 ( goto end )
roslaunch ..\main.launch models:=4
echo Finished the test launch file
Set /A Sleep+=1
echo %Sleep%
goto start
:end
echo "FINISHED!!"