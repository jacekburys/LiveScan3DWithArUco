/* A simple server in the internet domain using TCP
   The port number is passed as an argument */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>

int sockfd;

void error(const char *msg)
{
  perror(msg);
  exit(1);
}

void endProgram(int sig)
{
  close(sockfd);
  exit(0);
}

int main(int argc, char *argv[])
{
  int sockfd, connected;
  socklen_t clilen;
  char buffer[256];
  struct sockaddr_in serv_addr, cli_addr;
  int n;
  uint16_t port = 4800;
  int opt_val = 1;

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  setsockopt (sockfd, SOL_SOCKET, SO_REUSEADDR, (char *)&opt_val, sizeof(opt_val));

  if (sockfd < 0)
    error("ERROR opening socket");

  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(port);
  if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
    error("ERROR on binding");

  clilen = sizeof(cli_addr);
  listen(sockfd, 5);
  connected = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);

  if (connected < 0)
    error("ERROR on accept");

  signal(SIGINT, endProgram);

  int keepRunning = 1;

  while (keepRunning) {
    bzero(buffer,256);
    n = (int) read(connected, buffer, 255);
    if (n < 0) error("ERROR reading from socket");
    printf("%s", buffer);
  }
}