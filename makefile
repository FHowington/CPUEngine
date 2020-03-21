engine: main.o
	clang++ -o engine main.o  -lstdc++ -std=c++14 -lSDL2

main.o: main.cpp RasterizePolygon.h
	clang++ -c main.cpp -std=c++14

clean:
	rm main.o engine
