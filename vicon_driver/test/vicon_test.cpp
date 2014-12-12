#include "ViconDriver.h"
#include <unistd.h>
#include <signal.h>
#include <stdio.h>

ViconDriver vd;
bool running = false;

void sigint_handler(int signo)
{
  printf("Shutting down vicon driver\n");
  vd.shutdown();
  running = false;
}

int main(int argc, char **argv)
{
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = sigint_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  if(vd.init("alkaline"))
  {
    running = vd.start();
  }
  else
  {
    printf("Error connecting to server\n");
  }

  while(running)
  {
    sleep(1);
  }

  return 0;
}
