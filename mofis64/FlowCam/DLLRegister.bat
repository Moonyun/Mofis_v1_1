rem switch current parent to Dll path
cd /d %~dp0

rem Register CLR Component
"%WINDIR%\Microsoft.NET\Framework64\v4.0.30319\regasm.exe" FlowCamLibrary.dll /tlb /nologo /codebase

pause