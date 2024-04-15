
build:
	g++ main.cpp -Ofast -Wall

run: rebuild
	a.exe

rebuild: clean build

clean:
	del a.exe
	del image.png