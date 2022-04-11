all:  mavlink_control

mavlink_control: offline_test.cpp
	g++ -w -I mavlink/include/mavlink/v2.0 offline_test.cpp serial_port.cpp gimbal_interface.cpp -o gSDK -lpthread

git_submodule:
	git submodule update --init --recursive

clean:
	 rm -rf *o gSDK
