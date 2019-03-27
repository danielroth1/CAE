#include "SGNodeVisitorFactory.h"

SGNodeVisitor* SGNodeVisitorFactory::createPrinterVisitor(std::ostream& stream)
{
    class PrinterVisitor : public SGNodeVisitor
    {
    public:
        PrinterVisitor(std::ostream& stream)
            : _s(stream)
        {
        }

        void visit(SGChildrenNode* childrenNode)
        {
            print(childrenNode);
        }

        void visit(SGLeafNode* leafNode)
        {
            print(leafNode);
        }

        void print(SGNode* node)
        {
            for (size_t i = 0; i < node->calculateDepth(); ++i)
                _s << "  ";
            _s << node->getName() << "\n";
        }
        std::ostream& _s;
    };
    return new PrinterVisitor(stream);
}

SGNodeVisitorFactory::SGNodeVisitorFactory()
{

}
