SRC=../../src
test: test_main.cpp
	g++ -std=gnu++11 -Wall -Wextra -O1 -g -fsanitize=address,undefined -DHOST_TEST -I$(SRC) test_main.cpp \
	  $(SRC)/Rtcm.cpp $(SRC)/Rtcm1005.cpp $(SRC)/GnssParser.cpp $(SRC)/RtcmBus.cpp -o t && ./t
