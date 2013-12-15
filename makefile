all:
	mkdir -p  build
	cd build && cmake .. -G "MSYS Makefiles"
	cd build && make
	
clean:
	cd build && make clean