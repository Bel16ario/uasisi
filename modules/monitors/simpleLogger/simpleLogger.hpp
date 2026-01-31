#ifndef SIMPLELOGGER_HPP
#define SIMPLELOGGER_HPP

#include "uasisi/core/module.hpp"
#include "uasisi/core/types.hpp"
#include "uasisi/io/config.hpp"
#include <highfive/H5File.hpp>
#include <highfive/H5DataSet.hpp>
#include <highfive/H5DataSpace.hpp>
#include <string>
#include <vector>
#include <map>

namespace uasisi{
//I would like monitors to be a little different than other modules. When declaring signals, it should declare what signal types it supports and it is up to the orchestrator to decide which are given. I will signal this to the signal manager with empty signal names.

struct signalDatasetDOB{

    const SpanwiseVec<double>* dataPtr = nullptr;
    HighFive::DataSet coordsDataset;
    HighFive::DataSet dataDataset;
    size_t size;
    bool coordsWritten = false;

};

struct signalDatasetVEC{

    const SpanwiseVec<std::vector<double>>* dataPtr = nullptr;
    HighFive::DataSet coordsDataset;
    HighFive::DataSet dataDataset;
    size_t size; //I am adding these vars to ensure weird resizing does not ocur if the vectors are somehow resized when the module is running.
    size_t dims;
    bool coordsWritten = false; //Kind of redundant in the current implementation but maybe some day I add logic to handle changing coords.

};

struct signalDatasetSCA{

    const double* dataPtr = nullptr;
    HighFive::DataSet dataDataset;

};

class SimpleLogger : public IMonitor{ //Does not support airfoil input right now.

    public:

    SimpleLogger();
    ~SimpleLogger() override = default;

    void init() override;
    std::vector<SignalInfo> declareSignals() override;
    void validateConnections() override;

    void connectInputDouble(const std::string& name, const SpanwiseVec<double>* x) override;
    void connectInputVector(const std::string& name, const SpanwiseVec<std::vector<double>>* x) override;
    void connectInputScalar(const std::string& name, const double* x) override;

    void step(double t, double dt) override;

    const std::string& getName() const override { return this->name; }

    void setFileName(const std::string& nameNew);
    void setWriteInterval(size_t intervalNew);
    void setChunks(size_t chunksNew);

    private:

    std::string name = "simpleLogger";

    void generateFileName();
    void createFile();
    void createDatasets();
    void writeToDatasets(double t);

    std::string fileName;
    std::string baseName = "simulation";
    std::unique_ptr<HighFive::File> file;

    HighFive::DataSet timeData;

    size_t totalSteps = 0;
    size_t stepsWritten = 0; //Would be cool to implement flushInterval and wrtitesSinceFlush but for now it is not a priority
    size_t stepsSinceWrite = 0;
    size_t writeInterval = 5;
    size_t chunks = 100;

    bool isSet = false;
    bool isConnected = false;

    std::map<std::string, signalDatasetDOB> yellowPagesDOB; //This could be done by adding a ptr member to the SignalInfo class which would allow for more generic code in modules. This is too much work for me to change right now. Maybe later.
    std::map<std::string, signalDatasetVEC> yellowPagesVEC;
    std::map<std::string, signalDatasetSCA> yellowPagesSCA;

};

}
#endif
