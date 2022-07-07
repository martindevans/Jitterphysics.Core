@echo off

rem clean up any files / folders
if not exist output mkdir output
del /S /Q output
for /D %%p in ("output\*") do rmdir "%%p" /s /q


rem build 3D library
msbuild source\Jitter.Core.sln /p:Configuration=Release /t:Rebuild

rem ready for packaging
if not exist output\netstandard2.0\ mkdir output\netstandard2.0\

copy source\Jitter.Core\bin\Release\netstandard2.0\Jitter.Core.dll output\netstandard2.0\
copy source\Jitter.Core\bin\Release\netstandard2.0\Jitter.Core.pdb output\netstandard2.0\
copy source\Jitter.Core\bin\Release\netstandard2.0\Jitter.Core.xml output\netstandard2.0\
rem package
nuget pack nuget\Jitter.Core.nuspec -OutputDirectory output
