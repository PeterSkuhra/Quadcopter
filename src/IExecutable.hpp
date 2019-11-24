#ifndef IEXECUTABLE_HPP
#define IEXECUTABLE_HPP

class IExecutable
{
 public:
    virtual ~IExecutable() = default;

    virtual void Run() = 0;
};

#endif
