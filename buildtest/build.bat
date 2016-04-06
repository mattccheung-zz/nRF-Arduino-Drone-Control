ren *.c *.cpp
del protocol.cpp

for %%f in (*.cpp) do (
	FOR /f %%i IN ("%%~nf") DO (

	  ECHO NULL > %%~ni.h
	)
            
)