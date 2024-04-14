
build:
	g++ main.cpp

rebuild: clean build

clean:
	del a.exe
	del image.png