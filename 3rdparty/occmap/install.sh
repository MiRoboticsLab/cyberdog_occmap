rm -rf install
if [ ! -d "build" ]; then
 mkdir build
fi
cd build && rm -rf CMakeCache.txt
cmake .. && make -j4 && make install
cd ../ && rm -rf build