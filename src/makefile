CXX = clang++
CXXFLAGS = -lstdc++ -std=c++17 -lSDL2

all: CXXFLAGS += -Ofast -mavx2 -mfma -Rpass-analysis=loop-vectorize -Rpass=loop-vectorize
all: engine

debug: CXXFLAGS += -O0 -g -mavx2 -mfma -DDEBUG
debug: engine

slow: CXXFLAGS += -Ofast -mno-avx
slow: engine

slowdebug: CXXFLAGS += -O0 -g -DDEBUG
slowdebug: engine

engine: main.o tgaimage.o loader.o rasterize.o geometry.o shader.o pool.o
	$(CXX) $(CXXFLAGS) -o engine main.o tgaimage.o loader.o rasterize.o geometry.o pool.o

main.o: main.cpp rasterize.o loader.o Window.h shader.o pool.o
rasterize.o: rasterize.cpp rasterize.h loader.o Window.h geometry.o shader.o pool.o
loader.o: loader.cpp loader.h Window.h geometry.o tgaimage.h
geometry.o: geometry.cpp geometry.h Window.h
shader.o: shader.cpp shader.h
tgaimage.o: tgaimage.cpp tgaimage.h
pool.o: pool.cpp pool.h

clean:
	rm main.o tgaimage.o loader.o rasterize.o geometry.o pool.o engine
