#ifndef __EMOTIONINTERFACE__
#define __EMOTIONINTERFACE__

 // std
#include <string>
#include "SerialDeviceDriver.h"

namespace emotions {
  
  typedef struct
  {  
    std::string name;
    std::string leb;
    std::string reb;
    std::string mou;
    
  } EM_CODE;
  
  class EmotionInterface{
    
  private:
    // table with the setting for each emotion - from config file
    EM_CODE* emotion_table_;
    
    // number of emotions in the library
    int n_emotions_;

    // Driver for serial device
    SerialDeviceDriver driver_;
    
    bool writePort(const char* cmd);

    bool setupSerial( const char* config_file);
    bool readSerialConfig( SerialDeviceDriverSettings &config_serial, const char* config_file);

    bool setupEmotion(const char* config_file);
    bool build_emotion_lib( const char* config_file);

    int getIndex(const std::string cmd);
    
    bool send(const char *cmdbuffer);

  public:
    
    EmotionInterface();
    ~EmotionInterface();
    
    bool configure( const char* config_serial, const char* config_emotions );
    
    // interface functions
    bool setLeftEyebrow(const std::string  cmd);
    bool setRightEyebrow(const std::string cmd);
    bool setMouth(const std::string cmd);
    bool setAll(const std::string cmd);
    bool setRaw(const std::string cmd);
    
    // cycle through all emotions
    bool cycleAll();

    // say Hello 
    bool sayHello();

    // raise left eyebrow
    bool raiseEyebrow();

    // select random emotion
    bool randomEmotion( );

  };
  
}



#endif
