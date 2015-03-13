#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>

int main(void)
{
  int listenfd = 0,connfd = 0;
  
  struct sockaddr_in serv_addr;
 
  char sendBuff[1025];  
  int numrv;  
 
  listenfd = socket(AF_INET, SOCK_STREAM, 0);
  printf("socket retrieve success\n");
  
  memset(&serv_addr, '0', sizeof(serv_addr));
  memset(sendBuff, '0', sizeof(sendBuff));
      
  serv_addr.sin_family = AF_INET;    
  serv_addr.sin_addr.s_addr = htonl(INADDR_ANY); 
  serv_addr.sin_port = htons(4998);    
 
  bind(listenfd, (struct sockaddr*)&serv_addr,sizeof(serv_addr));
  
  if(listen(listenfd, 10) == -1){
      printf("Failed to listen\n");
      return -1;
  }
     

  connfd = accept(listenfd, (struct sockaddr*)NULL ,NULL); // accept awaiting request

  strcpy(sendBuff, "Message from server");
  write(connfd, sendBuff, strlen(sendBuff));

  close(connfd);    
  sleep(1);

  close(listenfd );
 
  return 0;
}
