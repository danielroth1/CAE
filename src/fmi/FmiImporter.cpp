//#include "FmiImporter.h"
//#include "FMUData.h"

//#include <stdio.h>
//#include <stdlib.h>
//#include <stdarg.h>

//#define BUFFER 1000

//FmiImporter::FmiImporter()
//{

//}

//void importlogger(jm_callbacks* c, jm_string module, jm_log_level_enu_t log_level, jm_string message)
//{
//    printf("module = %s, log level = %s: %s\n", module, jm_log_level_to_string(log_level), message);
//}

///* Logger function used by the FMU internally */

//void fmilogger(fmi2_component_t c, fmi2_string_t instanceName, fmi2_status_t status, fmi2_string_t category, fmi2_string_t message, ...)
//{
//    /* int len;
//    char msg[BUFFER]; */
//    va_list argp;
//    va_start(argp, message);
//    /* len = jm_vsnprintf(msg, BUFFER, message, argp); */
//    fmi2_log_forwarding_v(c, instanceName, status, category, message, argp);
//    va_end(argp);
//}

//int FmiImporter::executeTest(int argc, char *argv[])
//{
//    if (argc < 3)
//    {
//        printf("Usage: %s <fmu_file> <temporary_dir>\n", argv[0]);
//        return -1;
//    }
//    for (int k = 0; k < argc; k ++)
//        printf("argv[%d] = %s\n", k, argv[k]);

//    const char* FMUPath = argv[1];
//    const char* tmpPath = argv[2];

//    FMUData* fmuData = importFMU(FMUPath, tmpPath);
//    if (!fmuData)
//        return -1;

//    initiateInstance(fmuData, "Test CS model instance",
//                     0.0, 10.0);

//    fmi2_real_t tstart = 0.0;
//    fmi2_real_t tend = 10.0;
//    fmi2_real_t hstep = 0.01;
//    fmi2_real_t tcur = tstart;

//    while (tcur < tend)
//    {
//        doStep(fmuData, tcur, hstep);
//        tcur += hstep;
//    }

//    terminateInstance(fmuData);

//    freeFMU(fmuData);

//    return 0;
//}

//FMUData* FmiImporter::importFMU(
//        const char* FMUPath,
//        const char* tmpPath)
//{
//    fmi_version_enu_t version;
//    jm_status_enu_t status;

//    fmi2_import_t* fmu;


//    mCallbacks.malloc = malloc;
//    mCallbacks.calloc = calloc;
//    mCallbacks.realloc = realloc;
//    mCallbacks.free = free;
//    mCallbacks.logger = importlogger;
//    mCallbacks.log_level = jm_log_level_debug;
//    mCallbacks.context = 0;

//    mContext = fmi_import_allocate_context(&mCallbacks);

//    version = fmi_import_get_fmi_version(mContext, FMUPath, tmpPath);

//    if(version != fmi_version_2_0_enu) {
//        printf("The code only supports version 2.0\n");
//        return nullptr;
//    }

//    fmu = fmi2_import_parse_xml(mContext, tmpPath, 0);

//    if(!fmu) {
//        printf("Error parsing XML, exiting\n");
//        return nullptr;
//    }

//    if(fmi2_import_get_fmu_kind(fmu) == fmi2_fmu_kind_me) {
//        printf("Only CS 2.0 is supported by this code\n");
//        return nullptr;
//    }

//    mCallBackFunctions.logger = fmi2_log_forwarding;
//    mCallBackFunctions.allocateMemory = calloc;
//    mCallBackFunctions.freeMemory = free;
//    mCallBackFunctions.componentEnvironment = fmu;

//    status = fmi2_import_create_dllfmu(fmu, fmi2_fmu_kind_cs, &mCallBackFunctions);
//    if (status == jm_status_error) {
//        printf("Could not create the DLL loading mechanism(C-API) (error: %s).\n", fmi2_import_get_last_error(fmu));
//        return nullptr;
//    }
//    FMUData* fmuData = new FMUData();
//    fmuData->setContext(mContext);
//    fmuData->setFMU(fmu);

//    fmi2_string_t fmuGUID = fmi2_import_get_GUID(fmu);
//    printf("GUID:      %s\n", fmuGUID);

//    fmuData->setFMUGUID(fmuGUID);
//    return nullptr;
//}

//bool FmiImporter::initiateInstance(
//        FMUData* fmuData,
//        std::string instanceName,
//        fmi2_real_t tstart,
//        fmi2_real_t tend)
//{
//    fmi2_status_t fmistatus;
//    jm_status_enu_t jmstatus;

//    fmi2_string_t fmuLocation = "";
//    fmi2_boolean_t visible = fmi2_false;
//    fmi2_real_t relativeTol = 1e-4;
///*	fmi2_boolean_t loggingOn = fmi2_true; */

//    /* fmi2_real_t simulation_results[] = {-0.001878, -1.722275}; */
//    fmi2_real_t simulation_results[] = {0.0143633,   -1.62417};
//    fmi2_value_reference_t compare_real_variables_vr[] = {0, 1};
//    fmi2_boolean_t StopTimeDefined = fmi2_false;

//    if (sizeof(compare_real_variables_vr)/sizeof(fmi2_value_reference_t) != sizeof(simulation_results)/sizeof(fmi2_real_t)) {
//        printf("Number of simulation values and reference values are different\n");
//        return false;
//    }

//    fmi2_import_t* fmu = fmuData->getFMU();
//    printf("Version returned from FMU:   %s\n", fmi2_import_get_version(fmu));
//    printf("Platform type returned:      %s\n", fmi2_import_get_types_platform(fmu));

//    jmstatus = fmi2_import_instantiate(fmu, instanceName.c_str(), fmi2_cosimulation, fmuLocation, visible);
//    if (jmstatus == jm_status_error) {
//        printf("fmi2_import_instantiate failed\n");
//        return false;
//    }

//        fmistatus = fmi2_import_setup_experiment(fmu, fmi2_true,
//            relativeTol, tstart, StopTimeDefined, tend);
//    if(fmistatus != fmi2_status_ok) {
//        printf("fmi2_import_setup_experiment failed\n");
//        return false;
//    }

//        fmistatus = fmi2_import_enter_initialization_mode(fmu);
//    if(fmistatus != fmi2_status_ok) {
//        printf("fmi2_import_enter_initialization_mode failed\n");
//        return false;
//    }

//        fmistatus = fmi2_import_exit_initialization_mode(fmu);
//    if(fmistatus != fmi2_status_ok) {
//        printf("fmi2_import_exit_initialization_mode failed\n");
//        return false;
//    }
//    return true;
//}

//bool FmiImporter::terminateInstance(FMUData* fmuData)
//{
//    fmi2_import_t* fmu = fmuData->getFMU();
//    fmi2_status_t fmistatus = fmi2_import_terminate(fmu);
//    fmi2_import_free_instance(fmu);
//    return true;
//}

//bool FmiImporter::doStep(FMUData* fmuData,
//                         fmi2_real_t tcur,
//                         fmi2_real_t hstep)
//{
//    fmi2_import_t* fmu = fmuData->getFMU();

//    fmi2_boolean_t newStep = fmi2_true;
//    fmi2_status_t fmistatus =
//            fmi2_import_do_step(fmu, tcur, hstep, newStep);

//    // print every variable?

//    tcur += hstep; // TODO: this must be done after the method call
//    return true;
//}

////    test_simulate_cs(fmu);


//bool FmiImporter::freeFMU(FMUData* fmuData)
//{
//    fmi2_import_t* fmu = fmuData->getFMU();
//    fmi_import_context_t* context = fmuData->getContext();

//    fmi2_import_destroy_dllfmu(fmu);

//    fmi2_import_free(fmu);
//    fmi_import_free_context(context);

//    return true;
//}

