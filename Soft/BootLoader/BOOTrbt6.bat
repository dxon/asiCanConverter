echo off
rem echo.
echo File's merge: Application and Bootloader
echo Insert Serial Key 0x00000039
rem Вставка серийного номера. Если номре 0xFE5235A7, 
rem то записываем его: -repeat-data 0xA7 0x35 0x52 0xFE
d:\SRecord\srec_cat.exe D:\Projects\Ink_System\Soft\BootLoader\Release\Exe\boot.hex --Intel --exclude 0x8000D00 0x8000D04 -generator 0x8000D00 0x8000D04 -repeat-data 0x7D 0x02 0x00 0x00 --Output D:\Projects\Ink_System\Soft\BootLoader\Release\Exe\Bootrbt6.hex --Intel
echo Succeful complete!
C:\Progra~2\SEGGER\JLink_V498\JFlash.exe -openprjD:\Projects\Ink_System\Soft\BootLoader\Boot.jflash -openD:\Projects\Ink_System\Soft\BootLoader\Release\Exe\Bootrbt6.hex -connect -unsecurechip -erasechip -programverify -securechip -startapp -exit
IF ERRORLEVEL 1 goto ERROR
echo Programming succeful complete!
goto END
:ERROR
echo J-FlashARM Error!
:END
