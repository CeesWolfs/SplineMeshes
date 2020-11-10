CXXFLAGS = -std=c++20 -Wall -Wextra

all: MeshSplines
all: CXXFLAGS += -O3
clean:
	rm MeshSplines

debug: CXXFLAGS += -DDEBUG -g
debug: MeshSplines
profile: CXXFLAGS += -g -O2
profile: MeshSplines

MeshSplines: main.cpp Mesh.hpp SubEdgeTree.hpp Types.hpp
	$(CXX) -o $@ $< $(CXXFLAGS)
