@echo off
cls
Set Sleep=0
:start
if %Sleep% == 20 ( goto end )
roslaunch ..\main.launch algtype:=a-star models:=2
echo Finished the test launch file
Set /A Sleep+=1
echo %Sleep%
goto start
:end
echo "FINISHED!!"