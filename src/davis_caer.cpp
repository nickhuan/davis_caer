#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define PRINTF_LOG 1

/*caer*/
#include "nets.h"
#include "common.h"
#include "frame.h"

/*ROS node*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>

//using namespace std;
//using namespace cv;

int main(int argc, char *argv[])
{
	/********************TCP server********************/
	// First of all, parse the IP:Port we need to listen on.
	// Those are for now also the only two parameters permitted.
	// If none passed, attempt to connect to default TCP IP:Port.
	const char *ipAddress = "127.0.0.1";
	uint16_t portNumber = 7777;

	if (argc != 1 && argc != 3)
	{
		fprintf(stderr, "Incorrect argument number. Either pass none for default IP:Port"
			"combination of 127.0.0.1:7777, or pass the IP followed by the Port.\n");
		return (EXIT_FAILURE);
	}

	// If explicitly passed, parse arguments.
	if (argc == 3)
	{
		ipAddress = argv[1];
		sscanf(argv[2], "%" SCNu16, &portNumber);
	}

	// Create listening socket for TCP data.
	int listenTCPSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (listenTCPSocket < 0)
	{
		fprintf(stderr, "Failed to create TCP socket.\n");
		return (EXIT_FAILURE);
	}

	struct sockaddr_in listenTCPAddress;
	memset(&listenTCPAddress, 0, sizeof(struct sockaddr_in));

	listenTCPAddress.sin_family = AF_INET;
	listenTCPAddress.sin_port = htons(portNumber);
	inet_aton(ipAddress, &listenTCPAddress.sin_addr); // htonl() is implicit here.

	if (connect(listenTCPSocket, (struct sockaddr *) &listenTCPAddress, sizeof(struct sockaddr_in)) < 0)
	{
		fprintf(stderr, "Failed to connect to remote TCP data server.\n");
		return (EXIT_FAILURE);
	}

	// 64K data buffer should be enough for the TCP event packets.
	size_t dataBufferLength = 1024 * 64;
	uint8_t *dataBuffer = (uint8_t*)malloc(dataBufferLength);

	//The line somehow solved the bug: "Segmentation Fault (core dump)". Don't know why.
	cv::namedWindow ( "Davis frame" , 1 );

	/******************************image******************************/

	/*node for davis_caer*/
	ros::init(argc, argv, "davis_caer");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("davis_frame", 1);
//	ros::Rate loop_rate(1.0);

	while (ros::ok())
	{
		// Get packet header, to calculate packet size.
		if (!recvUntilDone(listenTCPSocket, dataBuffer, sizeof(struct caer_event_packet_header)))
		{
			fprintf(stderr, "Error in recv() call: %d\n", errno);
			break;
		}

		// Decode successfully received data.
		caerEventPacketHeader header = (caerEventPacketHeader) dataBuffer;

		uint16_t eventType = caerEventPacketHeaderGetEventType(header);
		uint16_t eventSource = caerEventPacketHeaderGetEventSource(header);
		uint32_t eventSize = caerEventPacketHeaderGetEventSize(header);
		uint32_t eventTSOffset = caerEventPacketHeaderGetEventTSOffset(header);
		uint32_t eventCapacity = caerEventPacketHeaderGetEventCapacity(header);
		uint32_t eventNumber = caerEventPacketHeaderGetEventNumber(header);
		uint32_t eventValid = caerEventPacketHeaderGetEventValid(header);

		printf("type = %" PRIu16 ", source = %" PRIu16 ", size = %" PRIu32 ", tsOffset = %" PRIu32 ", capacity = %" PRIu32 ", number = %" PRIu32 ", valid = %" PRIu32 ".\n",
			eventType, eventSource, eventSize, eventTSOffset, eventCapacity, eventNumber, eventValid);

		// Get rest of event packet, the part with the events themselves.
		if (!recvUntilDone(listenTCPSocket, dataBuffer + sizeof(struct caer_event_packet_header),
			eventCapacity * eventSize))
		{
			fprintf(stderr, "Error in recv() call: %d\n", errno);
			break;
		}

		if (eventValid > 0)
		{
			void *firstEvent = caerGenericEventGetEvent(header, 0);
			void *lastEvent = caerGenericEventGetEvent(header, eventValid - 1);
			uint32_t firstTS = caerGenericEventGetTimestamp(firstEvent, header);
			uint32_t lastTS = caerGenericEventGetTimestamp(lastEvent, header);
			uint32_t tsDifference = lastTS - firstTS;

			printf("Time difference in packet: %" PRIu32 " (first = %" PRIu32 ", last = %" PRIu32 ").\n", tsDifference,
				firstTS, lastTS);

			//todo: for loop, iterate through all events
			//todo: check if event type is frame (4)
			if (eventType == 4)
			{
			uint16_t *pixels = caerFrameEventGetPixelArrayUnsafe((caer_frame_event*)firstEvent);
			cv::Mat img(180,240,CV_16UC1,pixels);
			int k = img.at<int>(90,120);
			printf("img[90,120]=%d",k);
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", img).toImageMsg();
			pub.publish(msg);
			ros::spinOnce();
			}
//			loop_rate.sleep();
//			ros::spin();

		}

		printf("\n\n");
	}

	// Close connection.
	close(listenTCPSocket);

	free(dataBuffer);

	return (EXIT_SUCCESS);
}
