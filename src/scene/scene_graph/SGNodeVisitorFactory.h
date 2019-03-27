#ifndef SGNODEVISITORFACTORY_H
#define SGNODEVISITORFACTORY_H

#include <iostream>
#include <scene/scene_graph/SGCore.h>

class SGNodeVisitorFactory
{
public:
    static SGNodeVisitor* createPrinterVisitor(std::ostream& stream);

protected:
    SGNodeVisitorFactory();
};

#endif // SGNODEVISITORFACTORY_H
