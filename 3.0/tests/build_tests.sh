cd src
bash ./build_gtest.sh
bash ./generate-mocks.sh
mkdir -p build
cd build
cmake ..  # Must use cmake version 3.10 or later
make
MAKE_RET=$?
cd ../..
exit "$MAKE_RET"