#ifndef __UTILTIES__
#define __UTILITIES__

#include <fstream>
#include <yaml-cpp/yaml.h>
#include <facial_expression/emotionInterface.h>

void operator >> (const YAML::Node& node, emotions::EM_CODE& emotion) {
  emotion.name = node["name"].as<std::string>();
  emotion.leb = node["left"].as<std::string>();
  emotion.reb = node["right"].as<std::string>();
  emotion.mou = node["mouth"].as<std::string>();
}

void operator >> (const YAML::Node& node, bool& v) {
  int val = node.as<int>();
  if(val>0)
    v = 1;
  else 
    v = 0;
}

void operator >> (const YAML::Node& node, const char *v) {
  std::string paritymode = node.as<std::string>();
  
  if( paritymode.compare("Even")==0)
    v="EVEN";
  else 
    v="ODD";
}

void operator >> (const YAML::Node& node, char *v) {
  std::string val = node.as<std::string>();

  sprintf(v, "%s", val.c_str());
}

void operator >> (const YAML::Node& node, unsigned char v) {
  int val = node.as<int>();
  
  v = (unsigned char)val;
}

#endif
