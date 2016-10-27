#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>
#include <netdb.h> 

int sockfd;

void error(const char *msg)
{
  perror(msg);
  exit(0);
}

void endProgram(int sig)
{
  close(sockfd);
  exit(0);
}

int main(int argc, char *argv[])
{
  int sockfd, n;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  int opt_val = 1;

  char buffer[256];
  uint16_t port = 4800;
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  setsockopt (sockfd, SOL_SOCKET, SO_REUSEADDR, (char *)&opt_val, sizeof(opt_val));
  if (sockfd < 0)
    error("ERROR opening socket");

  server = gethostbyname("127.0.0.1");
  if (server == NULL) {
    fprintf(stderr,"ERROR, no such host\n");
    exit(0);
  }

  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy(server->h_addr, (char *)&serv_addr.sin_addr.s_addr, (size_t) server->h_length);
  serv_addr.sin_port = htons(port);
  if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
    error("ERROR connecting");

  signal(SIGINT, endProgram);

  int keepRunning = 1;

  while (keepRunning) {
    bzero(buffer,256);
    fgets(buffer,255, stdin);
    n = (int) write(sockfd, buffer, strlen(buffer));
    if (n < 0)
      error("ERROR writing to socket");
    bzero(buffer,256);
  }
}


