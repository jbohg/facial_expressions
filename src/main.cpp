#include <iostream>
#include <facial_expression/emotionInterface.h>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

void my_handler(int s){
           printf("Caught signal %d\n",s);
           exit(1); 

}

int main(int argc, char **argv) {

  signal (SIGINT,my_handler);

  if(argc<3){
    std::cerr << "\33[32;40m Usage: " << argv[0] << " /path/to/serial/config.yaml /path/to/emotion/library.yaml \33[0m" << std::endl;
    return -1;
  } 
  
  emotions::EmotionInterface emo;
  if(!emo.configure(argv[1], argv[2]))
    return -1;

  int count = 0;
  while(true) {
    //    emo.raiseEyebrow();
    //emo.randomEmotion();
    
    if(count%2)
      emo.setAll("hap");
    else 
      emo.setAll("neu");
    count++;
    
    usleep(3000000);
  }
  
  return 0;
}
