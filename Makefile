PYTHON_VERSION := $(shell which python3)

build:
	catkin config --no-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=$(PYTHON_VERSION)
	catkin build

install:
	chmod +x install.sh
	./install.sh
