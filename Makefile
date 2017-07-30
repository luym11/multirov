DEPS=./include
CXXFLAGS=-I$(DEPS)

%.o: %.cpp $(DEPS)
	$(CXX) -c -o $@ $< $(CXXFLAGS)

test_coveragemap: ./test/test_coveragemap.o ./src/coveragemap.o
	${CXX} -o $@ $^ ${CXXFLAGS} -lboost_system