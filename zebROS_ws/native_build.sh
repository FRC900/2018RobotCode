catkin_make_isolated --install --use-ninja --build-space build_native --devel-space devel_native --install-space install_native -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF "$@"
