@echo off
chcp 65001 > nul
echo =======================================
echo     正在自动提交并推送到 GitHub...
echo =======================================
git add .
set msg=auto update: %date:~0,4%-%date:~5,2%-%date:~8,2% %time:~0,2%:%time:~3,2%:%time:~6,2%
git commit -m "%msg%"
git push origin main
echo =======================================
echo     代码已成功同步至 GitHub！
echo =======================================
pause
