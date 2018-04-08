DEPS=./include
CXXFLAGS=-I$(DEPS) -std=c++11

%.o: %.cpp $(DEPS)
	$(CXX) -c -o $@ $< $(CXXFLAGS)

test_coveragemap: ./test/test_coveragemap.o ./src/coveragemap.o
	${CXX} -o $@ $^ ${CXXFLAGS} -lboost_system

test_explore_algo: ./src/coveragemap.o ./src/explore_algo.o ./test/test_explore_algo.o 
	${CXX} -o $@ $^ ${CXXFLAGS} -lboost_system

test_random: ./test/test_random.o
	${CXX} -o $@ $^ ${CXXFLAGS} -lboost_system