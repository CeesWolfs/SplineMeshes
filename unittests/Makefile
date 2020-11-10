CXXFLAGS = -std=c++20 -Wall -Wextra
SRCS := $(shell find . -name "*.cpp")
OBJS := $(addsuffix .o,$(basename $(SRCS)))
all: test
all: CXXFLAGS += -g -DDEBUG
cov: test_cov
cov: CXXFLAGS += -DDEBUG -g -fprofile-arcs -ftest-coverage
test: $(OBJS)
	$(CXX) -o $@ $^
test_cov: $(OBJS)
		$(CXX) -o $@ $^ -lgcov --coverage
%.o: %.c Makefile
	$(CXX) $(CFLAGS) $< -o $@
clean:
	ls *.o | grep -v "main-test.o" | xargs -I {} rm {}
	rm *.gcov *.gcda *.gcno test test_cov
