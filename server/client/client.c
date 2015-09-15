#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h> 

#include <zmq.h>
#include <assert.h>

#include "../server.h"

#define BUFSIZE 1280 * 1024
#define CLIENT_TRIGGER '1'
#define CLIENT_EXIT '2'
#define CLIENT_ACK '3'
#define FRAME_HEADER 0x04

int send_generic(void *requester, char *sendBuff, int size);
int recv_generic(void *requester, void *msgBuffer, int size);
int send_commands(void *requester, char *msgBuffer, clientCommands cmds);


int main(int argc, char *argv[])
{
    char recvBuff[BUFSIZE];
    char msgBuffer[256];
    memset(recvBuff, 0, sizeof(recvBuff));
    int numFrames;

    if(argc != 2)
    {
        printf("\n Usage: %s <tcp://ip of server:port number> \n",argv[0]);
        return 1;
    } 

    void *context = zmq_ctx_new ();
    void *requester = zmq_socket (context, ZMQ_REQ);
    assert(requester);
    zmq_connect (requester, argv[1]);



    // Trigger readout from server
    // TODO: take this from the command line or a config file
    clientCommands cmds;
    cmds.threshold = 0;
    cmds.nexposures = 1;
    cmds.configure = 3;
    cmds.lower_bound = 4;
    cmds.upper_bound = 5;
    cmds.gain = 6;
    send_commands(requester, msgBuffer, cmds);

    // Get number of frames from server
    recv_generic(requester, &numFrames, 4);

    msgBuffer[0] = CLIENT_ACK;
    send_generic(requester, msgBuffer, 1);
    for (int i = 0; i < numFrames; i ++) {
        recv_generic(requester, recvBuff, BUFSIZE);
        msgBuffer[0] = CLIENT_ACK;
        send_generic(requester, msgBuffer, 1);
    }
    zmq_close (requester);
    zmq_ctx_destroy (context);
    return 0;
}

// TODO: make more robust
int recv_generic(void *requester, void *msgBuffer, int size) {
    int bytes_received;
    if ((bytes_received = zmq_recv(requester, msgBuffer, size, 0) ) < 0) {
        perror("recv_int");
        return -1;
    } else {
        return bytes_received;
    }
}

int send_commands(void *requester, char *msgBuffer, clientCommands cmds) {
    // copy commands into send buffer
    memcpy(msgBuffer, &cmds, sizeof(cmds));
    return send_generic(requester, msgBuffer, sizeof(cmds));
}

int send_generic(void *requester, char *sendBuff, int size) {
    int totalsent;
    if ((totalsent = zmq_send(requester, sendBuff, size, 0)) < 0) {
        perror("send_generic");
        return -1;
    } else {
        return 0;
    }
}
