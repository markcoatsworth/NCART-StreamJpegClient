all: stream_jpeg_client
	./stream_jpeg_client

stream_jpeg_client: openni_client_mem.o process_jpeg.o globals.o osc_handlers.o
	g++ -o stream_jpeg_client openni_client_mem.o process_jpeg.o globals.o osc_handlers.o `pkg-config opencv --cflags --libs` -lpthread -llo -w

client_mem.o: openni_client_mem.cpp
	g++ -Wall -c -o openni_client_mem.o openni_client_mem.cpp `pkg-config opencv --cflags --libs` -lpthread -w

process_jpeg.o: process_jpeg.cpp
	g++ -Wall -c -o process_jpeg.o process_jpeg.cpp `pkg-config opencv --cflags --libs` -lpthread -w

globals.o: globals.cpp
	g++ -Wall -c -o globals.o globals.cpp `pkg-config opencv --cflags --libs` -lpthread -w

osc_handlers.o: osc_handlers.cpp
	g++ -Wall -c -o osc_handlers.o osc_handlers.cpp -llo -w

clean:
	rm -rf *.o stream_jpeg_client
