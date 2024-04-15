
build:
	g++ main.cpp -Ofast -Wall

debug_rebuild: clean
	g++ main.cpp -Wall

rebuild: clean build

clean:
	del a.exe
	del image.png