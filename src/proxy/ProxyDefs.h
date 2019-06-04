#ifndef PROXYDEFS_H
#define PROXYDEFS_H

#include <multi_threading/Domain.h>
#include <multi_threading/Operation.h>

// Invokes a method in a different domain.
// \param className - the class name of object calling the method
// \param callee - shared pointer to the object calling the method
// \param function - name of the method to be called. The method is called by
//          the callee.
// \param domain - pointer to the Domain object
// \param ... - method arguments WITHOUT types
#define INVOKE_METHOD(className, callee, function, domain, ...) \
    domain->addOperation( \
    new Operation(std::bind([__VA_ARGS__](std::shared_ptr<className> s) { \
            s->function(__VA_ARGS__); }, \
                std::static_pointer_cast<className>(callee->shared_from_this()))));

// A thread safe call to the given function.
// If there is no domain, nothing happens.
// If the current thread is the Domains thread, the operation is called directly.
// If the current thread is not the Domains thread, a Operation is added to the
// Domain. This is because it is not safe to call the function in a thread that
// is different from the Domain.
//
// \param className = Simulation
// \param callee = mSimulation
// \param function = function name
// \param ... = list of function parameters
#define PROXY_FUNCTION_CALL(className, callee, function, ...) \
    if (callee->getDomain()) \
    { \
        if (callee->getDomain()->getThreadId() == std::this_thread::get_id()) \
        { \
            callee->function(__VA_ARGS__); \
        } \
        else \
        { \
            callee->getDomain()->addOperation( \
            new Operation(std::bind([__VA_ARGS__](std::shared_ptr<className> s) { \
                    s->function(__VA_ARGS__); }, \
                        std::static_pointer_cast<className>(callee->shared_from_this())))); \
        } \
    }

#define PL(...) __VA_ARGS__

// Use this makro in the header file. Do not create or use a cpp file.
// className = Simulation
// callee = mSimulation
// function = function name
#define PROXY_FUNCTION(className, callee, function, paramsDefs, paramsNoDefs) \
    void function(paramsDefs) \
    { \
        PROXY_FUNCTION_CALL(className, callee, function, paramsNoDefs) \
    }

// Use this makro in the cpp file. It requires a corresponding makro in the
// header file.
// proxyClassName = Simulation
#define PROXY_FUNCTION_CPP(proxyClassName, className, callee, function, ...) \
    void proxyClassName::function() \
    { \
        PROXY_FUNCTION_CALL(className, callee, function, __VA_ARGS__) \
    }

// Use this makro in the head file. It requires a corresponding
// PROXY_FUNCTION_CPP in the cpp file.
// className = Simulation
// callee = mSimulation
// function = function name
// ... = function parameters
#define PROXY_FUNCTION_H(className, callee, function, ...) \
    void function(__VA_ARGS__);

// Declares proxy classes.
// A proxy class implements some methods of the original class.
// Proxy classes are used to add operations to domains of the instance
// of the original class.
// The constructor takes the instance of the original class.
// The original class must implement the "getDomain()" method.
// The proxy class only contains method that are specified with the
// PROXY_FUNCTION macro. PROXY_CLASS and PROXY_FUNCTION macros are used
// in the following way:
//
// PROXY_CLASS(proxyClassName, className, callee,
//      PROXY_FUNCTION(className, callee, method name, PL(<parameters with type>), PL(<parameters wihtout type>)),
//      PROXY_FUNCTION(className, callee, method name, PL(<parameters with type>), PL(<parameters wihtout type>)),
//      ... )
//
// \param classNameProxy (e.g. SimulationProxy)
// \param className (e.g. Simulation)
// \param callee (e.g. mSimulation) can be chosen arbirarily, the only condition is that it is the same for
//          both the PROXY_CLASS macro and all the PROXY_FUNCTION macros.
// \param ... method macros (e.g. PROXY_FUNCTION(Simulation, mS, initialize, , ))
//
// To get a better idea how this is done, have a look at SimulationControl.
#define PROXY_CLASS(classNameProxy, className, callee, ...) \
    class classNameProxy \
    { \
    public: \
        classNameProxy(className* a) \
        : callee(a) \
        { } \
        __VA_ARGS__ \
    private: \
        className* callee; \
    };

#define PROXY_CLASS_DERIVED(classNameProxy, className, derivedProxy, callee, ...) \
    class classNameProxy : public derivedProxy\
    { \
    public: \
        classNameProxy(className* a) \
        : derivedProxy(a) \
        , callee(a) \
        { } \
        __VA_ARGS__ \
    private: \
        className* callee; \
    };


#endif // PROXYDEFS_H
