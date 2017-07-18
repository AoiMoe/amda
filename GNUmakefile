CXX?=g++
CXXFLAGS?=-Wall -Wold-style-cast -O2 -std=c++14 -I.
ifeq ($(OS),Windows_NT)
EXESFX=.exe
endif
CLANG_FORMAT?= $(shell \
  if test -e .clang-format-executable-name; then \
    cat .clang-format-executable-name; \
  else \
    echo clang-format; \
  fi)

SIMPLE_TESTS =	amda_test1_separated.test \
		amda_test1_structured.test \
		amda_test1_delta_check.test \
		amda_test_failable.test
COMPLEX_TESTS =	amda_test2_separated.ctest \
		amda_test2_structured.ctest \
		amda_test2_delta_check.ctest

PROGS=	$(SIMPLE_TESTS:.test=$(EXESFX)) \
	$(COMPLEX_TESTS:.ctest=$(EXESFX)) \
	amda_test_failable_opt.s

FORMAT_SRCS=	amda.h \
		amda_test1.cpp \
		amda_test2.cpp \
		amda_test_failable.cpp

.PHONY: all clean format check-format check-clang-format-version test \

.DEFAULT: all
.SUFFIXES: .test .ctest

all: $(PROGS)

test: $(SIMPLE_TESTS)

%.test:
	./$(@:.test=)

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

format: check-clang-format-version
	$(CLANG_FORMAT) -style file -i $(FORMAT_SRCS)

check-format: check-clang-format-version
	@REPLACE=`for i in ${FORMAT_SRCS}; do \
	  $(CLANG_FORMAT) -style file -output-replacements-xml $$i | \
	    grep "<replacement " > /dev/null && echo $$i; \
	done`; \
	if test x"$$REPLACE" != x ; then \
	  echo need to run \"make format\" for: >&2; \
	  for i in $$REPLACE; do echo "  " $$i >&2; done; \
	  exit 1; \
	fi

check-clang-format-version:
	@if $(CLANG_FORMAT) --version | grep -qv '3\.9'; then \
		echo clang-format version error. >&2; \
		echo -n "expected 3.9.x, but " >&2 ; \
		$(CLANG_FORMAT) --version >&2; \
		exit 1; \
	fi
