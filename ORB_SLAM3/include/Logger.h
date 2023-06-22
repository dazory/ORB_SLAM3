#ifndef LOGGER_H
#define LOGGER_H

#include<list>
#include <string>


// namespace ORB_SLAM3
// {
class Logger
{

public:
    Logger();
    Logger(int iCur);// When its initialization the first map is created
    ~Logger(){};
    
    void begin(const std::string strVal, std::list<int> targets);
    void end(const std::string strVal, std::list<int> targets);
    void only_from(const std::string strVal, std::list<int> targets);
    void only(const std::string strVal, std::list<int> targets);
    void only_to(const std::string strVal, std::list<int> targets);
    
    void log_loc(const std::string strVal, std::list<int> targets);
    void bp(std::list<int> targets);
    void log_str(const std::string strVal, std::list<int> targets);
    int iCur = -1;
    std::string prefix = "";
    bool bOnly = false;

protected:

}; // class Logger

extern class Logger logger;
// } // namespace ORB_SLAM3

#endif // LOGGER_H
