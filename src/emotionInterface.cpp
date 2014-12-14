#include <facial_expression/emotionInterface.h>
#include <facial_expression/utilities.h>

#include <fstream>

namespace emotions {
  EmotionInterface::EmotionInterface()
  {}


  EmotionInterface::~EmotionInterface()
  {
    if (emotion_table_ != NULL)
      {
	delete [] emotion_table_;
	emotion_table_ = NULL;
      }
  }

  bool EmotionInterface::configure( const char* config_serial, const char* config_emotions )
  {
    if(!setupSerial(config_serial))
      return false;
    
    if(!setupEmotion(config_emotions))
      return false;
      
    return true;
  }

  bool EmotionInterface::setupEmotion(const char* config_file)
  {
    // set up emotion library from file
    if(!build_emotion_lib( config_file)){
      std::cerr << "\33[0;40m There was an error during parsing the emotion libray file. \33[0m" << std::endl;
      return false;
    }
  
    return true;
  }

  // set up emotion library
  bool EmotionInterface::build_emotion_lib( const char* config_file) 
  {
    std::cout << "Reading Emotion Library" << std::endl;
    try {
    
      std::ifstream fin(config_file);
      YAML::Parser parser(fin);
    
      YAML::Node doc;
      parser.GetNextDocument(doc);
    
      n_emotions_ = doc.size();
      emotion_table_ = new EM_CODE[n_emotions_];
      
      for(unsigned i=0;i<doc.size();i++) {
	doc[i] >> emotion_table_[i];
	std::cout << emotion_table_[i].name << "\n";
      }    

    } catch(YAML::ParserException& e) {
      std::cerr << "\33[32;40m" << e.what() << "\33[0m" << std::endl;
      return false;
    }
   
    return true;
  }

  bool EmotionInterface::setupSerial(const char* config_file)
  {
    SerialDeviceDriverSettings config;
    if(!readSerialConfig( config, config_file )) {
      std::cerr << "\33[0;40m There was an error during parsing the config for the serial port. \33[0m" << std::endl;
      return false;
    }
  
    //SerialDeviceDriver driver;
    if(driver_.open(config))
      std::cout << "Opened Serial Port" << std::endl;
    else {
      std::cerr << "\33[32;40m" << "Failed to open Serial Port" << "\33[0m"  << std::endl;
      return false;
    }
  
    return true;
  }



  bool EmotionInterface::readSerialConfig( SerialDeviceDriverSettings &config_serial, const char* config_file)
  {
    try {
    
      std::ifstream fin(config_file);
      YAML::Parser parser(fin);
    
      YAML::Node doc;
      parser.GetNextDocument(doc);
    
      doc["portname"] >> config_serial.CommChannel;
      doc["verbose"] >> config_serial.verbose;
      doc["baudrate"] >> config_serial.SerialParams.baudrate;
      doc["xonlim"] >> config_serial.SerialParams.xonlim;
      doc["xofflim"] >> config_serial.SerialParams.xofflim;
      doc["readmincharacters"] >> config_serial.SerialParams.readmincharacters;
      doc["readtimeoutmsec"] >> config_serial.SerialParams.readtimeoutmsec;
      doc["paritymode"] >> config_serial.SerialParams.paritymode;
      doc["ctsenb"] >> config_serial.SerialParams.ctsenb;
      doc["rtsenb"] >> config_serial.SerialParams.rtsenb;
      doc["xinenb"] >> config_serial.SerialParams.xinenb;
      doc["xoutenb"] >> config_serial.SerialParams.xoutenb;
      doc["modem"] >> config_serial.SerialParams.modem;
      doc["rcvenb"] >> config_serial.SerialParams.rcvenb;
      doc["dsrenb"] >> config_serial.SerialParams.dsrenb;
      doc["dtrdisable"] >> config_serial.SerialParams.dtrdisable;
      doc["databits"] >> config_serial.SerialParams.databits;
      doc["stopbits"] >> config_serial.SerialParams.stopbits;


    } catch(YAML::ParserException& e) {
      std::cerr << "\33[32;40m" << e.what() << "\33[0m" << std::endl;
      return false;
    }

    return true;
  }
  


  bool EmotionInterface::send(const char *cmdbuffer)
  {
    if (!driver_.send(cmdbuffer))
      return false;
    return true;
  }

  

  //bool writePort(const char* cmd);

  //get the index in _emotions_table of a emotion name
  int EmotionInterface::getIndex(const std::string cmd)
  {
    if(n_emotions_ == 0) {
      std::cerr << "\33[0;40m No Emotions known. \33[0m" << std::endl;
      return -1;
    }
 
    int i;
    for(i = 0; i < n_emotions_; i++)
      {
        if(emotion_table_[i].name.compare(cmd) == 0) //strings identical
	  break;
      }
    
    if( i == n_emotions_ ){
      std::cerr << "\33[0;40m No matching emotions found. \33[0m" << std::endl;
      return -1;
    }
    
    return i;
}

  
  // interface functions
  bool EmotionInterface::setLeftEyebrow(const std::string  cmd)
  {
    char cmdbuffer[] = {0,0,0,0};
    int i; 
    i = getIndex(cmd);
    if(i < 0)
      return false;
    
    if( emotion_table_[i].leb[0] == '-' || emotion_table_[i].leb[1] == '-') 
      return true;  //leave it in the same state
    
    cmdbuffer[0]= 'L';
    cmdbuffer[1]=emotion_table_[i].leb[0];
    cmdbuffer[2]=emotion_table_[i].leb[1];
    
    if(!send(cmdbuffer)){
      std::cerr << "\33[0;40m Sending failed. \33[0m" << std::endl;
      return false;
    }
    return true;
  }
  
  
  bool EmotionInterface::setRightEyebrow(const std::string cmd)
  {
    char cmdbuffer[] = {0,0,0,0};
    int i; 
    i = getIndex(cmd);
    if(i < 0)
      return false;
    
    if( emotion_table_[i].reb[0] == '-' || emotion_table_[i].reb[1] == '-') 
      return true;  //leave it in the same state
    
    cmdbuffer[0]= 'R';
    cmdbuffer[1]=emotion_table_[i].reb[0];
    cmdbuffer[2]=emotion_table_[i].reb[1];
    if(!send(cmdbuffer)){
      std::cerr << "\33[0;40m Sending failed. \33[0m" << std::endl;
      return false;
    }
    return true;
  }

  bool EmotionInterface::setMouth(const std::string cmd)
  {
    char cmdbuffer[] = {0,0,0,0};
    int i; 
    i = getIndex(cmd);
    if(i < 0)
      return false;
    
    if( emotion_table_[i].mou[0] == '-' || emotion_table_[i].mou[1] == '-') 
      return true;  //leave it in the same state
    
    cmdbuffer[0]= 'M';
    cmdbuffer[1]=emotion_table_[i].mou[0];
    cmdbuffer[2]=emotion_table_[i].mou[1];
    
    if(!send(cmdbuffer)){
      std::cerr << "\33[0;40m Sending failed. \33[0m" << std::endl;
      return false;
    }
    return true;

  }

  bool EmotionInterface::setAll(const std::string cmd)
  {
    if(!setLeftEyebrow(cmd)){
      std::cerr << "\33[0;40m Command not known to left eyebrow:" << cmd << "\33[0m" << std::endl;
      return false;
    }

    if(!setRightEyebrow(cmd)){
      std::cerr << "\33[0;40m Command not known to right eyebrow:" << cmd << "\33[0m" << std::endl;
      return false;
    }

    if(!setMouth(cmd)){
      std::cerr << "\33[0;40m Command not known to mouth:" << cmd << "\33[0m" << std::endl;
      return false;
    }
    
    return true;
  }

  bool EmotionInterface::setRaw(const std::string cmd)
  {
    if(!send(cmd.c_str())){
      std::cerr << "\33[0;40m Sending failed. \33[0m" << std::endl;
      return false;
    }
  
    return true;
  }

  bool EmotionInterface::raiseEyebrow( )
  {

    setAll("hap");

    usleep(50000);
    setRaw("L04");
    usleep(50000);
    setRaw("L08");
    usleep(50000);
    setRaw("L08");
    usleep(5000);
    setRaw("L04");
    usleep(5000);
    setRaw("L02");
    usleep(5000);
    setRaw("L01");

    return true;
  }

  bool EmotionInterface::randomEmotion( )
  {
    int r = (rand() % (n_emotions_));
    std::cout << "Being " << emotion_table_[r].name << std::endl;
    return setAll(emotion_table_[r].name);
    
    
  }
  

  bool EmotionInterface::cycleAll( )
  {
    for( int i=0; i<n_emotions_; ++i) {
      std::cout << "Being " << emotion_table_[i].name << std::endl;
      setAll(emotion_table_[i].name);
      usleep(5000000);
    }
    return true;
  }

  bool EmotionInterface::sayHello( )
  {
    int sleep = 200000;
    setAll("smi");
    usleep(sleep);
    setAll("hel");
    usleep(sleep);
    setAll("llo");
    usleep(sleep);
    setAll("smi");
    usleep(sleep);

    return true;
  }
 
}
