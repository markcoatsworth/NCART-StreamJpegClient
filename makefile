all: StreamJpegClient
	./StreamJpegClient

StreamJpegClient: StreamJpegClient.o process_jpeg.o globals.o osc_handlers.o
	g++ -o StreamJpegClient StreamJpegClient.o process_jpeg.o globals.o osc_handlers.o `pkg-config opencv --cflags --libs` -lpthread -llo -w

StreamJpegClient.o: StreamJpegClient.cpp
	g++ -Wall -c -o StreamJpegClient.o StreamJpegClient.cpp `pkg-config opencv --cflags --libs` -lpthread -w

process_jpeg.o: process_jpeg.cpp
	g++ -Wall -c -o process_jpeg.o process_jpeg.cpp `pkg-config opencv --cflags --libs` -lpthread -w

globals.o: globals.cpp
	g++ -Wall -c -o globals.o globals.cpp `pkg-config opencv --cflags --libs` -lpthread -w

osc_handlers.o: osc_handlers.cpp
	g++ -Wall -c -o osc_handlers.o osc_handlers.cpp -llo -w

clean:
	rm -rf *.o StreamJpegClient
