
build:
	g++ main.cpp -Ofast -flto -funroll-loops -finline-functions -march=native -Wall

run: rebuild
	a.exe

rebuild: clean build

debug: clean
	g++ main.cpp -Og -Wall -Wextra -Wpedantic -I .stb_image_write.h

clean:
	del a.exe
	del image.png