engine: main.o tgaimage.o loader.o RasterizePolygon.o
	clang++ -o engine main.o tgaimage.o loader.o RasterizePolygon.o -lstdc++ -std=c++17 -lSDL2 -g -mavx2 -O3 -Rpass-analysis=loop-vectorize -Rpass=loop-vectorize

main.o: main.cpp RasterizePolygon.o loader.o Window.h
	clang++ -c main.cpp -std=c++17 -g  -mavx2 -O3 -Rpass-analysis=loop-vectorize -Rpass=loop-vectorize

tgaimage.o: tgaimage.cpp tgaimage.h
	clang++ -c tgaimage.cpp -std=c++17 -g -mavx2 -O3 -Rpass-analysis=loop-vectorize -Rpass=loop-vectorize

RasterizePolygon.o: RasterizePolygon.cpp RasterizePolygon.h loader.o Window.h
	clang++ -c RasterizePolygon.cpp -std=c++17 -g -mavx2 -O3 -Rpass-analysis=loop-vectorize -Rpass=loop-vectorize

loader.o: loader.cpp loader.h Window.h
	clang++ -c loader.cpp -std=c++17 -g -O3 -mavx2 -O3 -Rpass-analysis=loop-vectorize -Rpass=loop-vectorizexs

clean:
	rm main.o tgaimage.o loader.o RasterizePolygon.o engine
