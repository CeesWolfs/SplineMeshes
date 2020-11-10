#!/bin/bash
make clean
make cov
rm -rf coverage/
./test_cov
ls *.cpp | grep -v "main-test.cpp" | xarg {} gcov {}
lcov -c --directory . --output-file coverage.info
genhtml coverage.info --output coverage
firefox coverage/index.html
