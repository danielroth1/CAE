#ifndef DEMO_H
#define DEMO_H


#include <string>

// A demo is a class that is loaded once its corresponding entry in the
// list view of the DemoLoader is clicked. It is unloaded if a different
// demo is loaded.
class Demo
{
public:
    Demo();
    virtual ~Demo();

    virtual std::string getName() = 0;

    // Loads the demo. Is called when the corresponding entry in the
    // list view is clicked.
    virtual void load() = 0;

    // Unloads the demo. This method is only called after load() is called.
    // Unloads all resources from the scene.
    // Is called when a different demo is loaded or the scene is cleared.
    virtual void unload() = 0;
};

#endif // DEMO_H
