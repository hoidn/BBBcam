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

#define BUFSIZE 1280 * 1024
#define CLIENT_TRIGGER '1'
#define CLIENT_EXIT '2'
#define CLIENT_ACK '3'
#define FRAME_HEADER 0x04

// command codes for client to pass parameters
#define CMD_THRESHOlD '5'
#define CMD_NEXPOSURES '7'
#define CMD_CONFIG '8'
#define CMD_BOUND_LOWER '9'
#define CMD_BOUND_UPPER 'a'
#define CMD_GAIN 'b'


int read_size = 30;
int numFrames = 10;
char sendBuff[BUFSIZE];

typedef struct{
    uint32_t threshold;
    uint32_t nexposures;
    uint32_t gain;
    uint32_t configure;
    uint32_t lower_bound;
    uint32_t upper_bound;
} clientCommands;

int handle_msg(int connfd, char *msgBuffer);
int send_int(int connfd, int *msgBuffer);
int trigger_exposure();
int exposure();
int mysend_image(int connfd);
int recv_commands(int connfd, char *msgBuffer, clientCommands cmds);


int main(int argc, char *argv[])
{
    int listenfd = 0, connfd = 0;
    struct sockaddr_in serv_addr; 
    clientCommands cmds;

    char msgBuffer[256];
    time_t ticks; 
    ssize_t bytes_received;

    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serv_addr, '0', sizeof(serv_addr));
    memset(sendBuff, '0', sizeof(sendBuff)); 

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(5001); 

    bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)); 

    listen(listenfd, 10); 

    connfd = accept(listenfd, (struct sockaddr*)NULL, NULL); 
    ticks = time(NULL);
//    if ((bytes_received = handle_msg(connfd, msgBuffer)) <= 0) {
//        exit(EXIT_FAILURE);
//    }
//    if (msgBuffer[0] == CLIENT_TRIGGER) {
//        printf("triggered\n");
//    } else {
//        printf("unexpected code received\n");
//    }

    if (recv_commands(connfd, msgBuffer, cmds) < 0) {
        exit(EXIT_FAILURE);
    } else {
        printf("triggered\n");
    }

    // Send number of frames to client
    send_int(connfd, &numFrames);
    // get response
    if ((bytes_received = handle_msg(connfd, msgBuffer)) <= 0) {
        exit(EXIT_FAILURE);
    }
    if (msgBuffer[0] == CLIENT_ACK) {
        printf("client ack after frame count sent\n");
    } else {
        printf("unexpected code received\n");
    }
    trigger_exposure();
    exposure();
    // TODO: move above to for loop
    for (int i = 0; i < numFrames; i ++) {
        if (mysend_image(connfd) < 0) {
            exit(EXIT_FAILURE);
        }
        if ((bytes_received = handle_msg(connfd, msgBuffer)) <= 0) {
            exit(EXIT_FAILURE);
        }
        if (msgBuffer[0] == CLIENT_ACK) {
            printf("client ack\n");
            msgBuffer[0] = '0';
        } else {
            printf("unexpected code received\n");
        }
    }

    close(connfd);
    close(listenfd);
}


int handle_msg(int connfd, char *msgBuffer) {
    // Expects a single-byte response from the client in the socket buffer
    // Returns the size of the message received (i.e., 1 on success)
    int bytes_received;
    if ((bytes_received = recv(connfd, msgBuffer, 1, 0) ) <= 0) {
        if (bytes_received == 0) {
            printf("server: socket %d hung up\n", connfd);
        } else {
            perror("recv");
        }
        close(connfd);
        return -1;
    } else {
        // got some data from client
        return bytes_received;
    }
}

int recv_commands(int connfd, char *msgBuffer, clientCommands cmds) {
    // Reads a sequence of single-byte commands (with variable-size payloads)
    // from the socket buffer and writes them into an instance of the
    // clientCommands struct
    // Returns 0 on success and -1 on failure
    int bytes_received = 0;
    int chunk_size = 0;
    while (bytes_received < read_size) {
        if ((chunk_size = recv(connfd, &msgBuffer[bytes_received], read_size - bytes_received, 0) ) <= 0) {
            if (chunk_size == 0) {
                printf("server: socket %d hung up\n", connfd);
            } else {
                perror("recv");
            }
            close(connfd);
            return -1;
        } else {
            // got some data from client
            bytes_received += chunk_size;
            if (bytes_received > read_size) {
                printf("recv_commands: read more characters than expected");
            }
        }
    }
    // pack the commands into cmds
    int parsed_size = 0;
    while (parsed_size < read_size) {
        char cmd = msgBuffer[parsed_size];
        switch(cmd) {
            case CMD_THRESHOlD:
                cmds.threshold = ((uint32_t) (msgBuffer[parsed_size + 1]));
            case CMD_NEXPOSURES:
                cmds.nexposures = ((uint32_t) (msgBuffer[parsed_size + 1]));
            case CMD_CONFIG:
                cmds.configure = ((uint32_t) (msgBuffer[parsed_size + 1]));
            case CMD_BOUND_LOWER:
                cmds.lower_bound = ((uint32_t) (msgBuffer[parsed_size + 1]));
            case CMD_BOUND_UPPER:
                cmds.upper_bound = ((uint32_t) (msgBuffer[parsed_size + 1]));
            case CMD_GAIN:
                cmds.gain = ((uint32_t) (msgBuffer[parsed_size + 1]));
            default:
                printf("unexpected command\n");
        }
        printf("%c\n", msgBuffer);
        printf("%u\n", (uint32_t) (&msgBuffer[parsed_size + 1]));
        parsed_size += 5;
    }
    return 0;
}

int trigger_exposure() {
    return 0;
}

// 
int exposure() {
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

int mysend_image(int connfd) {
    int totalsent = 0;
    int sent;
    while (totalsent < BUFSIZE) {
        if ((sent = send(connfd, sendBuff, sizeof(sendBuff), 0)) == 0) {
            perror("socket connection broken on send attempt");
            close(connfd);
            return -1;
        }
        totalsent += sent;
    }
    return 0;
}

int send_int(int connfd, int *msgBuffer) {
    int totalsent = 0;
    int sent;
    while (totalsent < sizeof(int)) {
        if ((sent = send(connfd, msgBuffer, sizeof(int), 0)) <= 0) {
            perror("socket connection broken on send attempt");
            close(connfd);
            return -1;
        }
        totalsent += sent;
    }
    return 0;
}
