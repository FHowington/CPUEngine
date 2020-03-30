engine: main.o
	clang++ -o engine main.o  -lstdc++ -std=c++17 -lSDL2 -g

main.o: main.cpp RasterizePolygon.h loader.h Window.h
	clang++ -c main.cpp -std=c++17 -g

clean:
	rm main.o engine
