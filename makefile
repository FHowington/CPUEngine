engine: main.o tgaimage.o loader.o RasterizePolygon.o geometry.o
	clang++ -o engine main.o tgaimage.o loader.o RasterizePolygon.o geometry.o -lstdc++ -std=c++17 -lSDL2 -g -mavx2 -Ofast -Rpass-analysis=loop-vectorize -Rpass=loop-vectorize -mfma

main.o: main.cpp RasterizePolygon.o loader.o Window.h
	clang++ -c main.cpp -std=c++17 -g  -mavx2 -Ofast -Rpass-analysis=loop-vectorize -Rpass=loop-vectorize -mfma

tgaimage.o: tgaimage.cpp tgaimage.h
	clang++ -c tgaimage.cpp -std=c++17 -g -mavx2 -Ofast -Rpass-analysis=loop-vectorize -Rpass=loop-vectorize

RasterizePolygon.o: RasterizePolygon.cpp RasterizePolygon.h loader.o Window.h geometry.o
	clang++ -c RasterizePolygon.cpp -std=c++17 -g -mavx2 -Ofast -Rpass-analysis=loop-vectorize -Rpass=loop-vectorize -mfma

loader.o: loader.cpp loader.h Window.h
	clang++ -c loader.cpp -std=c++17 -g -Ofast -mavx2 -Ofast -Rpass-analysis=loop-vectorize -Rpass=loop-vectorizexs -mfma

geometry.o: geometry.cpp geometry.h Window.h
	clang++ -c geometry.cpp -std=c++17 -g -Ofast -mavx2 -Ofast -Rpass-analysis=loop-vectorize -Rpass=loop-vectorizexs -mfma

clean:
	rm main.o tgaimage.o loader.o RasterizePolygon.o geometry.o engine
