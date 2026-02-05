#include "simpleLogger.hpp"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <highfive/H5File.hpp>
#include <highfive/H5DataSet.hpp>
#include <highfive/H5DataSpace.hpp>
#include <string>

namespace uasisi{

SimpleLogger::SimpleLogger(){
    std::cout << "Simple Logger Monitor created\n";
    this->generateFileName();
}

void SimpleLogger::init(){

    if(!this->isConnected){
        throw std::runtime_error("Module not connected/validated");
    }

    this->createFile();
    if(!std::filesystem::exists(this->fileName)){
        throw std::runtime_error("Error creating log file");
    } else {
        std::cout << "Output file created:\t" << this->fileName << "\n";
    }
    this->createDatasets();
    this->isSet = true;
    this->writeToDatasets(0.0);

}

std::vector<SignalInfo> SimpleLogger::declareSignals(){
    
    std::vector<SignalInfo> reqSignals;
    reqSignals.reserve(3);
    reqSignals.push_back(SignalInfo("", DataType::DOB, SignalType::IN)); //Is there a better way to make the string be empty?
    reqSignals.push_back(SignalInfo("", DataType::VEC, SignalType::IN));
    reqSignals.push_back(SignalInfo("", DataType::SCA, SignalType::IN));

    return reqSignals;

}

void SimpleLogger::validateConnections(){ //A lot of the validation is actually done in the connect methods and the private methods. Not much to do here I believe.
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already set or connected");
    }
    if(this->yellowPagesDOB.empty() && this->yellowPagesVEC.empty() && this->yellowPagesSCA.empty()){
        std::cout << "WARNING: module unconnected\n";
    }
    this->isConnected = true;

}

void SimpleLogger::connectInputDouble(const std::string& name, const SpanwiseVec<double>* x){ //Not sure if it is better to check for duplicate signals here on in the validation method

    if(this->isConnected || this->isSet){
        throw std::runtime_error("Module already connected or initialized");
    }
    bool cond = std::any_of(this->inputs.begin(), this->inputs.end(), [&name](const SignalInfo& s){return s.name() == name && s.dataType() == DataType::DOB;});
    if(cond || this->yellowPagesDOB.find(name) != this->yellowPagesDOB.end()){
        throw std::runtime_error("Duplicate signal");
    }
    signalDatasetDOB xSignalDataset;
    xSignalDataset.dataPtr = x;
    this->yellowPagesDOB[name] = xSignalDataset;
    this->inputs.push_back(SignalInfo(name, DataType::DOB, SignalType::IN));
    std::cout << "Connected input: " << name << std::endl;

}

void SimpleLogger::connectInputVector(const std::string& name, const SpanwiseVec<std::vector<double>>* x){

    if(this->isConnected || this->isSet){
        throw std::runtime_error("Module already connected or initialized");
    }
    bool cond = std::any_of(inputs.begin(), inputs.end(), [&name](const SignalInfo& s){return s.name() == name && s.dataType() == DataType::VEC;});
    if(cond || this->yellowPagesVEC.find(name) != this->yellowPagesVEC.end()){
        throw std::runtime_error("Duplicate signal");
    }
    signalDatasetVEC xSignalDataset;
    xSignalDataset.dataPtr = x;
    this->yellowPagesVEC[name] = xSignalDataset;
    this->inputs.push_back(SignalInfo(name, DataType::VEC, SignalType::IN));
    std::cout << "Connected input: " << name << std::endl;

}

void SimpleLogger::connectInputScalar(const std::string& name, const double* x){

    if(this->isConnected || this->isSet){
        throw std::runtime_error("Module already connected or initialized");
    }
    bool cond = std::any_of(inputs.begin(), inputs.end(), [&name](const SignalInfo& s){return s.name() == name && s.dataType() == DataType::SCA;});
    if(cond || this->yellowPagesSCA.find(name) != this->yellowPagesSCA.end()){
        throw std::runtime_error("Duplicate signal");
    }
    signalDatasetSCA xSignalDataset;
    xSignalDataset.dataPtr = x;
    this->yellowPagesSCA[name] = xSignalDataset;
    this->inputs.push_back(SignalInfo(name, DataType::SCA, SignalType::IN));
    std::cout << "Connected input: " << name << std::endl;

}

void SimpleLogger::step(double t, double dt){

    if(!this->isConnected || !this->isSet){
        throw std::runtime_error("Module not connected/validated or initalized yet");
    }
    
    if(this->stepsSinceWrite >= this->writeInterval){
        this->writeToDatasets(t);
        this->stepsSinceWrite = 1;
    } else {
        this->stepsSinceWrite++;
    }
    this->totalSteps++;

}

void SimpleLogger::setFileName(const std::string& nameNew){ //This method could be much simpler by just changing the baseName and running generateFileName afterwards. However, I will leave it for now just to keep baseName as is.
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    if (nameNew.length() < 3 || nameNew.substr(nameNew.length() - 3) != ".h5") {
        this->fileName = nameNew + ".h5";
    } else {
        this->fileName = nameNew;
    }
    if(std::filesystem::exists(this->fileName)){
        std::cout << "WARNING: file already exists, adding suffix\n";
        std::stringstream nameStream;
        size_t suffix = 1;
        std::string nameWOExt = this->fileName.substr(0, this->fileName.length() - 3);
        do {
            nameStream.str("");
            nameStream.clear();
            nameStream << nameWOExt << "_" << suffix << ".h5";
            this->fileName = nameStream.str();
            suffix++;
        } while (std::filesystem::exists(this->fileName));
    }
}

void SimpleLogger::setWriteInterval(size_t intervalNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->writeInterval = intervalNew;

}

void SimpleLogger::setChunks(size_t chunksNew){
    if(this->isSet || this->isConnected){
        throw std::runtime_error("Module already connected");
    }
    this->chunks = chunksNew;

}

void SimpleLogger::generateFileName(){ //No damage here without flag checks I think

    std::time_t time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream nameStream;
    nameStream << this->baseName << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S") << ".h5";
    this->fileName = nameStream.str();
    if(std::filesystem::exists(this->fileName)){
        size_t suffix = 1;
        std::string nameWOExt = this->fileName.substr(0, this->fileName.length() - 3);
        do {
            nameStream.str("");
            nameStream.clear();
            nameStream << nameWOExt << "_" << suffix << ".h5";
            this->fileName = nameStream.str();
            suffix++;
        } while (std::filesystem::exists(this->fileName));
    }
}

void SimpleLogger::createFile(){

    if(this->isSet){
        throw std::runtime_error("Module already initialized");
    }
    this->file = std::make_unique<HighFive::File>(this->fileName, HighFive::File::Create);
    //Here, metadata should be added. For now, I will leave it empty 

}

std::string SimpleLogger::getFileName(){ //I should add another common Module method run at the end of simulation. In the case of this module for example, it could print the fileName
    return this->getFileName();
}

void SimpleLogger::createDatasets(){
    if(this->isSet){
        throw std::runtime_error("Module already initialized");
    }
    if(!std::filesystem::exists(this->fileName)){
        throw std::runtime_error("File has not been created");
    }
    HighFive::DataSpace timeSpace({0}, {HighFive::DataSpace::UNLIMITED});
    HighFive::DataSetCreateProps timeProps;
    timeProps.add(HighFive::Chunking({this->chunks}));
    this->timeData = this->file->createDataSet<double>("/time", timeSpace, timeProps);

    for(auto& [name, signal] : this->yellowPagesDOB){

        signal.size = signal.dataPtr->size();
        HighFive::DataSpace coordsSpace({signal.size});
        signal.coordsDataset = this->file->createDataSet<double>("/signals/" + name + "/coords", coordsSpace);
        signal.coordsDataset.write(signal.dataPtr->coords());
        signal.coordsWritten = true;

        HighFive::DataSpace dataSpace({0, signal.size}, {HighFive::DataSpace::UNLIMITED, signal.size});
        HighFive::DataSetCreateProps dataProps;
        dataProps.add(HighFive::Chunking({this->chunks, std::max(size_t(1), signal.size)}));
        signal.dataDataset = this->file->createDataSet<double>("/signals/" + name + "/data", dataSpace, dataProps);

    }
    
    for(auto& [name, signal] : this->yellowPagesVEC){
        
        signal.size = signal.dataPtr->size();
        if(signal.size == 0){
            throw std::runtime_error("Empty vector");
        }
        signal.dims = (*signal.dataPtr)[0].size();
        for(size_t i = 0; i < signal.size; i++){
            if((*signal.dataPtr)[i].size() != signal.dims){
                throw std::runtime_error("Uneven sizing!");
            }
            if((*signal.dataPtr)[i].size() == 0){
                throw std::runtime_error("Empty nested vector");
            }
        }

        HighFive::DataSpace coordsSpace({signal.size});
        signal.coordsDataset = this->file->createDataSet<double>("/signals/" + name + "/coords", coordsSpace);
        signal.coordsDataset.write(signal.dataPtr->coords());
        signal.coordsWritten = true;

        HighFive::DataSpace dataSpace({0, signal.size, signal.dims}, {HighFive::DataSpace::UNLIMITED, signal.size, signal.dims});
        HighFive::DataSetCreateProps dataProps;
        dataProps.add(HighFive::Chunking({this->chunks, std::max(size_t(1), signal.size), std::max(size_t(1), signal.dims)}));
        signal.dataDataset = this->file->createDataSet<double>("/signals/" + name + "/data", dataSpace, dataProps);

    }
    
    for(auto& [name, signal] : this->yellowPagesSCA){

        HighFive::DataSpace dataSpace({0}, {HighFive::DataSpace::UNLIMITED});
        HighFive::DataSetCreateProps dataProps;
        dataProps.add(HighFive::Chunking({this->chunks}));
        signal.dataDataset = this->file->createDataSet<double>("/signals/" + name + "/data", dataSpace, dataProps);

    }
}

void SimpleLogger::writeToDatasets(double t){
    
    if(!this->isSet || !this->isConnected){
        throw std::runtime_error("Module is not running");
    }
    if(!std::filesystem::exists(this->fileName)){
        throw std::runtime_error("File has not been created");
    }
    this->timeData.resize({this->stepsWritten + 1});
    this->timeData.select({this->stepsWritten}, {1}).write(t);

    for(auto& [name, signal] : this->yellowPagesDOB){
      
        std::vector<std::vector<double>> data2D = {signal.dataPtr->SWData()};
        signal.dataDataset.resize({(this->stepsWritten + 1), signal.size});
        signal.dataDataset.select({this->stepsWritten, 0}, {1, signal.size}).write(data2D);

    }
    
    for(auto& [name, signal] : this->yellowPagesVEC){
        
        std::vector<std::vector<std::vector<double>>> data3D = {signal.dataPtr->SWData()};
        signal.dataDataset.resize({(this->stepsWritten + 1), signal.size, signal.dims});
        signal.dataDataset.select({this->stepsWritten, 0, 0}, {1, signal.size, signal.dims}).write(data3D); //From HighFive documentation I think this works directly and I do not need to flatten it.

    }
    
    for(auto& [name, signal] : this->yellowPagesSCA){
        
        signal.dataDataset.resize({(this->stepsWritten + 1)});
        signal.dataDataset.select({this->stepsWritten}, {1}).write(std::vector<double>{*signal.dataPtr});

    }
    
    this->stepsWritten++;

}


}

namespace{
static uasisi::ModuleRegistration<uasisi::SimpleLogger> registrationSimpleLogger("simpleLogger");
}
