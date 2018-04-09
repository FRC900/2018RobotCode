catkin_make_isolated --install --use-ninja --build-space native_isolated --devel-space native_devel --install-space native_install -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF "$@"
