engine: main.o
	clang++ -o engine main.o -lstdc++ -std=c++17 -lSDL2 -O3 -g -Rpass-analysis=loop-vectorize -Rpass=loop-vectorize -mavx2

main.o: main.cpp RasterizePolygon.h loader.h Window.h
	clang++ -c main.cpp -std=c++17 -O3 -g -Rpass-analysis=loop-vectorize -Rpass=loop-vectorize -mavx2

clean:
	rm main.o engine
