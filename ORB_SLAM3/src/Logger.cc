#include<list>
#include <string>
#include <iostream>
#include <algorithm>
#include "Logger.h"



bool Contains(const std::list<int> &list, int x)
{
	return std::find(list.begin(), list.end(), x) != list.end();
}

// namespace ORB_SLAM3
// {

Logger::Logger(){

}
Logger::Logger(int iCur){
    this->iCur=iCur;
}

void Logger::log_loc(const std::string strVal, std::list<int> targets)
{
    if(Contains(targets, this->iCur)){
        std::cout << "\n" << "[" << strVal << "]" << "\n";
    }
}
void Logger::bp(std::list<int> targets)
{
    if(Contains(targets, this->iCur)){
        const std::string strVal = std::string(">>");
        std::cout << "---" << strVal << "---" << "\n"; std::cin.ignore(); std::cout << "\n\n";
    }
}
void Logger::log_str(const std::string strVal, std::list<int> targets)
{
    if(Contains(targets, this->iCur)){
        std::cout << strVal << std::endl;
    }
}

class Logger logger;

// } //namespace ORB_SLAM
