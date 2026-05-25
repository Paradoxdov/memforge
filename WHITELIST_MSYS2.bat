@echo off
:: Add C:\msys64 to Windows Defender exclusions so gcc/ld/etc can run.
:: Needs admin. Self-elevates via PowerShell if not already elevated.
:: Run once — Defender remembers the exclusion permanently after that.

setlocal

:: --- Check if we're admin ---
net session >nul 2>&1
if %errorlevel% neq 0 (
    echo Requesting administrator privileges...
    powershell -NoProfile -Command "Start-Process -FilePath '%~f0' -Verb RunAs"
    exit /b
)

echo.
echo ============================================
echo  Whitelisting C:\msys64 in Windows Defender
echo ============================================
echo.

:: --- Add the folder exclusion ---
echo [1/3] Adding folder exclusion: C:\msys64
powershell -NoProfile -Command ^
    "try { Add-MpPreference -ExclusionPath 'C:\msys64' -ErrorAction Stop; Write-Host '   OK' -ForegroundColor Green } catch { Write-Host ('   FAILED: ' + $_.Exception.Message) -ForegroundColor Red }"

:: --- Add specific exe exclusions (covers some Defender quirks) ---
echo.
echo [2/3] Adding process exclusion: gcc.exe + ld.exe + collect2.exe
powershell -NoProfile -Command ^
    "$exes = @('C:\msys64\mingw64\bin\gcc.exe','C:\msys64\mingw64\bin\ld.exe','C:\msys64\mingw64\bin\collect2.exe','C:\msys64\mingw64\libexec\gcc\x86_64-w64-mingw32\14.2.0\collect2.exe','C:\msys64\mingw64\libexec\gcc\x86_64-w64-mingw32\15.1.0\collect2.exe'); foreach ($e in $exes) { try { Add-MpPreference -ExclusionProcess $e -ErrorAction Stop; Write-Host ('   OK  ' + $e) -ForegroundColor Green } catch { Write-Host ('   skip ' + $e) -ForegroundColor DarkGray } }"

:: --- Show current exclusion list to confirm ---
echo.
echo [3/3] Verifying current exclusions...
powershell -NoProfile -Command ^
    "$p = Get-MpPreference; Write-Host ''; Write-Host 'Folder exclusions containing msys64:' -ForegroundColor Cyan; $p.ExclusionPath | Where-Object { $_ -like '*msys64*' } | ForEach-Object { '  ' + $_ }; Write-Host ''; Write-Host 'Process exclusions containing msys64:' -ForegroundColor Cyan; $p.ExclusionProcess | Where-Object { $_ -like '*msys64*' } | ForEach-Object { '  ' + $_ }"

echo.
echo ============================================
echo  Done. You can now run gcc/make without
echo  Defender / Smart App Control blocking it.
echo.
echo  If gcc STILL fails — Smart App Control is
echo  a separate switch from Defender exclusions.
echo  Disable it via:
echo    Windows Security -^> App ^& browser control
echo    -^> Smart App Control settings -^> Off
echo  (Requires a reboot. SAC cannot be re-enabled
echo   after that without a Windows reset.)
echo ============================================
echo.
pause
