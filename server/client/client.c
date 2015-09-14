#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h> 

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

typedef struct{
    uint32_t threshold;
    uint32_t nexposures;
    uint32_t gain;
    uint32_t configure;
    uint32_t lower_bound;
    uint32_t upper_bound;
} clientCommands;


int send_generic(int connfd, char *msgBuffer, int size);
int send_commands(int connfd, char *msgBuffer, clientCommands cmds);
int send_msg(int connfd, char *msgBuffer);
int recv_image(int connfd, int size);
int recv_int(int connfd, int *msgBuffer);

char recvBuff[BUFSIZE];
char msgBuffer[35];


int main(int argc, char *argv[])
{
    memset(recvBuff, 0, sizeof(recvBuff));
    int sockfd = 0, n = 0;
    int numFrames;
    struct sockaddr_in serv_addr; 

    if(argc != 2)
    {
        printf("\n Usage: %s <ip of server> \n",argv[0]);
        return 1;
    } 

    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Error : Could not create socket \n");
        return 1;
    } 

    memset(&serv_addr, '0', sizeof(serv_addr)); 

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(5001); 

    if(inet_pton(AF_INET, argv[1], &serv_addr.sin_addr)<=0)
    {
        printf("\n inet_pton error occured\n");
        return 1;
    } 

    if( connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
       printf("\n Error : Connect Failed \n");
       return 1;
    } 

//    while ( (n = read(sockfd, recvBuff, sizeof(recvBuff)-1)) > 0)
//    {
//        recvBuff[n] = 0;
//        if(fputs(recvBuff, stdout) == EOF)
//        {
//            printf("\n Error : Fputs error\n");
//        }
//    } 
//
//    if(n < 0)
//    {
//        printf("\n Read error \n");
//    } 

    // Trigger readout from server
    // msgBuffer[0] = CLIENT_TRIGGER;
    // send_msg(sockfd, msgBuffer);
    //
    // pack commands to send
    clientCommands cmds;
    cmds.threshold = 0;
    cmds.nexposures = 1;
    cmds.configure = 3;
    cmds.lower_bound = 4;
    cmds.upper_bound = 5;
    cmds.gain = 6;
    send_commands(sockfd, msgBuffer, cmds);

    // Get number of frames from server
    recv_int(sockfd, &numFrames);
    

    msgBuffer[0] = CLIENT_ACK;
    send_msg(sockfd, msgBuffer);
    for (int i = 0; i < numFrames; i ++) {
        recv_image(sockfd, BUFSIZE);
        msgBuffer[0] = CLIENT_ACK;
        send_msg(sockfd, msgBuffer);
    }
    return 0;
}

int send_msg(int connfd, char *msgBuffer) {
    int sent;
    if ((sent = send(connfd, msgBuffer, 1, 0)) <= 0) {
        perror("socket connection broken on send attempt");
        close(connfd);
        return -1;
    }
    return 0;
}

int recv_image(int connfd, int size) {
    int nchunk = 0;
    int bytes_received = 0;
    int chunk_size = 0;
    while (bytes_received < BUFSIZE) {
        if ((chunk_size = recv(connfd, recvBuff + bytes_received, size - bytes_received, 0) ) <= 0) {
            if (chunk_size == 0) {
                printf("server: socket %d hung up\n", connfd);
            } else {
                perror("recv");
            }
            close(connfd);
            return -1;
        } else {
            // got some data from client
            nchunk +=1;
            bytes_received += chunk_size;
            if (bytes_received == BUFSIZE) {
                printf("%d\n", nchunk);
                return bytes_received;
            }
        }
    }
}

// TODO: make more robust
int recv_int(int connfd, int *msgBuffer) {
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
        return bytes_received;
    }
}

int send_commands(int connfd, char *msgBuffer, clientCommands cmds) {
    int bytes_packed = 0;
    
    msgBuffer[0] = CMD_THRESHOlD;
    memcpy(&msgBuffer[1], (void *) &cmds.threshold, 4);
    msgBuffer[10] = CMD_CONFIG;
    memcpy(&msgBuffer[11], (void *) &cmds.configure, 4);
    msgBuffer[15] = CMD_BOUND_LOWER;
    memcpy(&msgBuffer[16], (void *) &cmds.lower_bound, 4);
    msgBuffer[20] = CMD_BOUND_UPPER;
    memcpy(&msgBuffer[21], (void *) &cmds.upper_bound, 4);
    msgBuffer[25] = CMD_GAIN;
    memcpy(&msgBuffer[26],(void *) &cmds.gain, 4);
    send_generic(connfd, msgBuffer, read_size);
}

int send_generic(int connfd, char *sendBuff, int size) {
    int totalsent = 0;
    int sent;
    while (totalsent < size) {
        if ((sent = send(connfd, &sendBuff[totalsent], size - totalsent, 0)) == 0) {
            perror("socket connection broken on send attempt");
            close(connfd);
            return -1;
        }
        totalsent += sent;
    }
    return 0;
}
