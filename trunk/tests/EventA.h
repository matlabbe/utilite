#ifndef EVENTA_H
#define EVENTA_H

#include "UEvent.h"
#include <string>

class EventA : public UEvent
{
public:
    EventA(const int &code, std::string msg) : UEvent(code), _msg(msg) {}
    virtual ~EventA() {}

    virtual std::string getClassName() const {return std::string("EventA");}

    const std::string &getMsg() const {return _msg;}

    enum {TEST};

protected:

private:
    std::string _msg;

};

#endif
