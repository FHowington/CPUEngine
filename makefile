CXX = clang++
CXXFLAGS = -lstdc++ -std=c++17 -lSDL2 -Rpass-analysis=loop-vectorize -Rpass=loop-vectorize

all: CXXFLAGS += -Ofast -mavx2 -mfma
all: engine

debug: CXXFLAGS += -O0 -g -mavx2 -mfma
debug: engine

slow: CXXFLAGS += -Ofast
slow: engine

engine: main.o tgaimage.o loader.o rasterize.o geometry.o
	$(CXX) $(CXXFLAGS) -o engine main.o tgaimage.o loader.o rasterize.o geometry.o

main.o: main.cpp rasterize.o loader.o Window.h
tgaimage.o: tgaimage.cpp tgaimage.h
rasterize.o: rasterize.cpp rasterize.h loader.o Window.h geometry.o
loader.o: loader.cpp loader.h Window.h geometry.o
geometry.o: geometry.cpp geometry.h Window.h

clean:
	rm main.o tgaimage.o loader.o rasterize.o geometry.o engine
