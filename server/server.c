#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <time.h> 

#include <zmq.h>
#include <assert.h>

#include "server.h"

#define BUFSIZE 1280 * 1024
#define CLIENT_TRIGGER '1'
#define CLIENT_EXIT '2'
#define CLIENT_ACK '3'
#define FRAME_HEADER 0x04


int numFrames = 10;


int handle_msg(void *responder, char *msgBuffer);
int send_int(void *responder, int *msgBuffer);
int send_image(void *responder, uint8_t *imgBuffer, int size);
int recv_commands(void *responder, char *msgBuffer, clientCommands cmds);
int trigger_exposure();
int exposure(uint8_t *sendBuff);


int main(int argc, char *argv[])
{
    // struct to pass around commands for the cam driver
    clientCommands cmds;

    uint8_t sendBuff[BUFSIZE];
    char msgBuffer[256];
    // time_t ticks; 
    ssize_t bytes_received;

    //  Socket to talk to clients
    void *context = zmq_ctx_new ();
    void *responder = zmq_socket (context, ZMQ_REP);
    int rc = zmq_bind (responder, "tcp://*:5555");
    assert (rc == 0);

    memset(sendBuff, '0', BUFSIZE); 

    // ticks = time(NULL);

    // Get camera command parameters from client
    if (recv_commands(responder, msgBuffer, cmds) < 0) {
        exit(EXIT_FAILURE);
    } else {
        printf("triggered\n");
    }

    // Send number of frames to client
    send_int(responder, &numFrames);
    // get response
    if ((bytes_received = handle_msg(responder, msgBuffer)) < 0) {
        exit(EXIT_FAILURE);
    }
    if (msgBuffer[0] == CLIENT_ACK) {
        printf("client ack after frame count sent\n");
    } else {
        printf("unexpected code received\n");
    }
    trigger_exposure();
    exposure(sendBuff);
    // TODO: move above to for loop
    for (int i = 0; i < numFrames; i ++) {
        if (send_image(responder, sendBuff, sizeof(sendBuff)) < 0) {
            exit(EXIT_FAILURE);
        }
        if ((bytes_received = handle_msg(responder, msgBuffer)) < 0) {
            exit(EXIT_FAILURE);
        }
        if (msgBuffer[0] == CLIENT_ACK) {
            printf("client ack\n");
            msgBuffer[0] = '0';
        } else {
            printf("unexpected code received\n");
        }
    }
    zmq_close (responder);
    zmq_ctx_destroy (context);
}


int handle_msg(void *responder, char *msgBuffer) {
    // Returns the size of the message received (i.e., 1 on success)
    int bytes_received;
    if ((bytes_received = zmq_recv(responder, msgBuffer, 1, 0) ) <= 0) {
        perror("handle_msg");
        return -1; 
    } else {
        // got some data from client
        return bytes_received;
    }
}

int recv_commands(void *responder, char *msgBuffer, clientCommands cmds) {
    // Reads a sequence of single-byte commands (with variable-size payloads)
    // from the socket buffer and writes them into an instance of the
    // clientCommands struct
    // Returns 0 on success and -1 on failure
    // Receive from socket into msgBuffer
    // printf("size of msgBuffer: %d\n", sizeof(cmds));
    int bytes_received = zmq_recv(responder, msgBuffer, sizeof(cmds), 0);
    if (bytes_received < 0) {
        perror("recv_commands");
        return -1;
    } 
    memcpy(&cmds, msgBuffer, sizeof(cmds));
    printf("%d\n", cmds.threshold);
    printf("%d\n", cmds.nexposures);
    printf("%d\n", cmds.configure);
    printf("%d\n", cmds.lower_bound);
    printf("%d\n", cmds.upper_bound);
    printf("%d\n", cmds.gain);
    printf("%d bytes received\n", bytes_received);
    return bytes_received;
}

int trigger_exposure() {
    return 0;
}

// 
int exposure(uint8_t *sendBuff) {
    FILE *f;
    f = fopen("ceil.7.28sum.dat", "rb");
    if (f) {
        fread(sendBuff, BUFSIZE, 1, f);
    } else {
        perror("error opening file");
        return -1;
    }
    return 0;
}

int send_image(void *responder, uint8_t *imgBuffer, int size) {
    int totalsent;
    if ((totalsent = zmq_send(responder, imgBuffer, size, 0)) <= 0) {
        perror("send_image");
        return -1;
    } else {
        printf("size sent: %d\n", totalsent);
        return totalsent;
    }
}

int send_int(void *responder, int *msgBuffer) {
    int totalsent;
    if ((totalsent = zmq_send(responder, msgBuffer, sizeof(int), 0)) <= 0) {
        perror("send_int");
        return -1;
    } else {
        return totalsent;
    }
}
