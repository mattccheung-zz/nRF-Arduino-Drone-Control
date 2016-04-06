ren *.c *.cpp
del protocol.cpp

for %%f in (*.cpp) do (
	FOR /f %%i IN ("%file%") DO (

  ECHO NULL > %%~ni.h
 )
            echo %%~nf
            process_in "%%~nf.in"
            process_out "%%~nf.out"
)