#ifndef BUFFER_H
#define BUFFER_H


class Buffer
{
public:
    Buffer();
    virtual ~Buffer();

    virtual void bind() = 0;

    virtual void unbind() = 0;
};

#endif // BUFFER_H
