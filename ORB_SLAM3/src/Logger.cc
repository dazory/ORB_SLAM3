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

void Logger::begin(const std::string strVal, std::list<int> targets)
{
    if(Contains(targets, this->iCur)){
        std::cout << "\n";
        std::cout << this->prefix << "Begin of [" << strVal << "]\n";
        this->prefix = "   " + this->prefix;
    }
}

void Logger::end(const std::string strVal, std::list<int> targets)
{
    if(Contains(targets, this->iCur)){
        std::cout << this->prefix << "End of [" << strVal << "]\n\n";
        this->prefix = this->prefix.erase(0, 3);
    }
}

void Logger::only_from(const std::string strVal, std::list<int> targets)
{
    if(Contains(targets, this->iCur) && this->bOnly){
        std::cout << this->prefix << strVal << std::endl;
        this->bOnly = true;
    }
}

void Logger::only(const std::string strVal, std::list<int> targets)
{
    if(Contains(targets, this->iCur) && this->bOnly){
        std::cout << this->prefix << strVal << std::endl;
    }
}

void Logger::only_to(const std::string strVal, std::list<int> targets)
{
    if(Contains(targets, this->iCur) && this->bOnly){
        std::cout << this->prefix << strVal << std::endl;
        this->bOnly = false;
    }
}

void Logger::log_loc(const std::string strVal, std::list<int> targets)
{
    if(Contains(targets, this->iCur)){
        std::cout << this->prefix << "[" << strVal << "]" << "\n";
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
        std::cout << this->prefix << strVal << std::endl;
    }
}

class Logger logger;

// } //namespace ORB_SLAM
