//#ifndef FMIIMPORTER_H
//#define FMIIMPORTER_H


//// Includes
//#include <fmilib.h>
//#include <JM/jm_portability.h>
//#include <string>

//// Forward Declarations
//class FMUData;

//class FmiImporter
//{
//public:
//    FmiImporter();
//    int executeTest(int argc, char* argv[]);

//    bool freeFMU(FMUData* fmuData);
//    FMUData* importFMU(const char* FMUPath, const char* tmpPath);

//    bool initiateInstance(FMUData* fmuData,
//                          std::string instanceName = "Test CS model instance",
//                          fmi2_real_t tstart = 0.0,
//                          fmi2_real_t tend = 0.0);

//    bool terminateInstance(FMUData* fmuData);

//    bool doStep(FMUData* fmuData,
//                fmi2_real_t tcur,
//                fmi2_real_t hstep = 0.1);

//private:

//    // Context
//    fmi_import_context_t* mContext;

//    fmi_version_enu_t mVersion;

//    // Callbacks
//    jm_callbacks mCallbacks;
//    fmi2_callback_functions_t mCallBackFunctions;
//};

//#endif // FMIIMPORTER_H
