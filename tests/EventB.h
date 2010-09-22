#ifndef EVENTB_H
#define EVENTB_H

#include "UEvent.h"
#include <string>

class EventB : public UEvent
{
public:
    EventB(const int &code, std::string msg) : UEvent(code), _msg(msg) {}
    virtual ~EventB() {}

    virtual std::string getClassName() const {return std::string("EventB");}

    const std::string &getMsg() const {return _msg;}

    enum {TEST};

protected:

private:
    std::string _msg;

};

#endif
