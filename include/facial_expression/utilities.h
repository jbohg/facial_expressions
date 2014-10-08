#ifndef __UTILTIES__
#define __UTILITIES__

#include <yaml-cpp/yaml.h>
#include <facial_expression/emotionInterface.h>

void operator >> (const YAML::Node& node, emotions::EM_CODE& emotion) {
  node["name"] >> emotion.name;
  node["left"] >> emotion.leb;
  node["right"] >> emotion.reb;
  node["mouth"] >> emotion.mou;
}

void operator >> (const YAML::Node& node, bool& v) {
  int val;
  node >> val;
  if(val>0)
    v = 1;
  else 
    v = 0;
}

void operator >> (const YAML::Node& node, const char *v) {
  std::string paritymode;
  node >> paritymode;
  
  if( paritymode.compare("Even")==0)
    v="EVEN";
  else 
    v="ODD";
}

void operator >> (const YAML::Node& node, char *v) {
  std::string val;
  node >> val;
  sprintf(v, "%s", val.c_str());
}

void operator >> (const YAML::Node& node, unsigned char v) {
  int val;
  node >> val;
  
  v = (unsigned char)val;
}

#endif
