
build:
	del render.exe
	g++ main.cpp -Ofast -flto -funroll-loops -finline-functions -march=native -o render.exe

run: clean build
	render.exe

debug:
	del debug_render.exe
	g++ main.cpp -Og -Wall -Wextra -Wpedantic -I .stb_image_write.h -o debug_render.exe

clean:
	del render.exe
	del debug_render.exe
	del crop.exe
	del image.png


cropper:
	del crop.exe
	g++ seam_carving_cropper.cpp -O3 -Wall -Wextra -Wpedantic -o crop.exe