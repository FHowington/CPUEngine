engine: main.o tgaimage.o loader.o
	clang++ -o engine main.o tgaimage.o loader.o -lstdc++ -std=c++17 -lSDL2 -O3 -g -Rpass-analysis=loop-vectorize -Rpass=loop-vectorize -mavx2

main.o: main.cpp RasterizePolygon.h loader.h Window.h
	clang++ -c main.cpp -std=c++17 -O3 -g -Rpass-analysis=loop-vectorize -Rpass=loop-vectorize -mavx2

tgaimage.o: tgaimage.cpp tgaimage.h
	clang++ -c tgaimage.cpp -std=c++17 -O3 -g -Rpass-analysis=loop-vectorize -Rpass=loop-vectorize -mavx2

loader.o: loader.cpp loader.h
	clang++ -c loader.cpp -std=c++17 -O3 -g -Rpass-analysis=loop-vectorize -Rpass=loop-vectorize -mavx2

clean:
	rm main.o tgaimage.o loader.o engine
