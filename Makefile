
build:
	g++ main.cpp -Ofast -flto -funroll-loops -finline-functions -march=native -Wall

run: rebuild
	a.exe

rebuild: clean build

clean:
	del a.exe
	del image.png