@echo off
REM Build script for ESP-IDF project
cd /d "c:\Users\bolsm\Documents\GitHub\LLLL"

REM Try to find ESP-IDF
if exist "C:\esp\v6.0\esp-idf\idf.py" (
    echo Found ESP-IDF at C:\esp\v6.0\esp-idf
    cd /d "C:\esp\v6.0\esp-idf"
    python idf.py build
) else if exist "C:\Program Files\IDF\idf.py" (
    echo Found ESP-IDF at C:\Program Files\IDF
    cd /d "C:\Program Files\IDF"
    python idf.py build
) else (
    echo ESP-IDF not found. Checking installed paths...
    dir C:\esp
    dir "C:\Program Files" /ad /b | findstr /i esp
)
