catkin_make_isolated --install --use-ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=`pwd`/rostoolchain.cmake -DCATKIN_ENABLE_TESTING=OFF "$@"
