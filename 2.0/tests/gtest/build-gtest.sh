WORKSPACE=gtest
mkdir -p ${WORKSPACE}
rm -rf ${WORKSPACE}/*
cd ${WORKSPACE}
#wget https://github.com/google/googletest/archive/release-1.7.0.zip
unzip ../release-1.7.0.zip
#rm release-1.7.0.zip
cd googletest*
mv * ..
cd ..
patch -p0 < ../GetThreadCountForLinux.patch
mkdir -p build
cd build
cmake ..
make
cd ..

# This is an environment script to be sourced by users of this tool
echo export GTEST_INCLUDE=${WORKSPACE}/include > environment_gtest-1.7.0
echo export GTEST_LIBS=${WORKSPACE}/build >> environment_gtest-1.7.0
chmod a+x environment_gtest-1.7.0
