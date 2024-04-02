
build:
	g++ main.cpp
	a.exe

rebuild: clean build

clean:
	del a.exe
	del image.png