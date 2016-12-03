CXX=g++
CXXFLAGS=-O2 -std=c++14 -I.
ifeq ($(OS),Windows_NT)
EXESFX=.exe
endif
CLANG_FORMAT?=clang-format

PROGS=	amda_test1_separated$(EXESFX) \
	amda_test1_structured$(EXESFX) \
	amda_test1_delta_check$(EXESFX) \
	amda_test2_separated$(EXESFX) \
	amda_test2_structured$(EXESFX) \
	amda_test2_delta_check$(EXESFX) \
	amda_test_failable$(EXESFX) \
	amda_test_failable_opt.s

FORMAT_SRCS=	amda.h \
		amda_test1.cpp \
		amda_test2.cpp \
		amda_test_failable.cpp

.PHONY: all clean
.DEFAULT: all

all: $(PROGS)

amda_test1_separated$(EXESFX): amda_test1.cpp amda.h
	$(CXX) $(CXXFLAGS) -DTEST_SEPARATED -o $@ $<

amda_test1_structured$(EXESFX): amda_test1.cpp amda.h
	$(CXX) $(CXXFLAGS) -DTEST_STRUCTURED -o $@ $<

amda_test1_delta_check$(EXESFX): amda_test1.cpp amda.h
	$(CXX) $(CXXFLAGS) -DTEST_DELTA_CHECK -o $@ $<

amda_test2_separated$(EXESFX): amda_test2.cpp amda.h
	$(CXX) $(CXXFLAGS) -DTEST_SEPARATED -o $@ $<

amda_test2_structured$(EXESFX): amda_test2.cpp amda.h
	$(CXX) $(CXXFLAGS) -DTEST_STRUCTURED -o $@ $<

amda_test2_delta_check$(EXESFX): amda_test2.cpp amda.h
	$(CXX) $(CXXFLAGS) -DTEST_DELTA_CHECK -o $@ $<

amda_test_failable$(EXESFX): amda_test_failable.cpp amda.h
	$(CXX) $(CXXFLAGS) -DSIDE_EFFECT -o $@ $<

amda_test_failable_opt.s: amda_test_failable.cpp amda.h
	$(CXX) $(CXXFLAGS) -S -o $@ $<

clean:
	rm -f $(PROGS)
	rm -f test1.da test2.da

format:
	$(CLANG_FORMAT) -style file -i $(FORMAT_SRCS)
