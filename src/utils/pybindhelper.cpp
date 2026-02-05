#include "uasisi/utils/pybindhelper.hpp"
#include <any>
#include <ios>
#include <iostream>
#include <stdexcept>
#include <string>

namespace uasisi{

PythonInterpreter& PythonInterpreter::getInstance(){ //I don't understand why I can't use this-> but removing it fixed the LSP's diagnosis
    if(!instance){
        instance = new PythonInterpreter();
    } else {
        std::cout << "WARNING: Interpreter already alive\n"; //Alert user to be careful to not cross code between modules.
    }
    return *instance;
}

void PythonInterpreter::destroyInstance(){
    delete instance;
    instance = nullptr;
}

void PythonConfigDict::setConfig(const std::string& key, const std::any& value){
    if(value.type() == typeid(int)){
        this->setConfigInt(key, std::any_cast<int>(value));
    } else if(value.type() == typeid(bool)){
        this->setConfigBool(key, std::any_cast<bool>(value));
    } else if(value.type() == typeid(double)){
        this->setConfigDouble(key, std::any_cast<double>(value));
    } else if(value.type() == typeid(std::string)){
        this->setConfigString(key, std::any_cast<std::string>(value));
    } else if(value.type() == typeid(py::object)){
        this->setConfigObject(key, std::any_cast<py::object>(value));
    } else {
        throw std::runtime_error("Invalid type");
    }
}

void PythonConfigDict::setConfigInt(const std::string& key, const int value){
    if(this->configDict.contains(key)){
        std::cout << "WARNING: Overwriting key " << key << " with value " << std::to_string(value) << "\n";
    }
    this->configDict[key] = value;
}

void PythonConfigDict::setConfigBool(const std::string& key, const bool value){
    if(this->configDict.contains(key)){
        std::cout << "WARNING: Overwriting key " << std::boolalpha << key << " with value " << std::to_string(value) << "\n";
    }
    this->configDict[key] = value;
}

void PythonConfigDict::setConfigDouble(const std::string& key, const double value){
    if(this->configDict.contains(key)){
        std::cout << "WARNING: Overwriting key " << key << " with value " << std::to_string(value) << "\n";
    }
    this->configDict[key] = value;
}

void PythonConfigDict::setConfigString(const std::string& key, const std::string& value){
    if(this->configDict.contains(key)){
        std::cout << "WARNING: Overwriting key " << key << " with value \"" << value << "\"\n";
    }
    this->configDict[key] = value;
}

void PythonConfigDict::setConfigObject(const std::string& key, const py::object& value){
    if(this->configDict.contains(key)){
        std::string typeName = py::str(value.get_type().attr("__name__"));
        std::string info;
        if (typeName == "function") {
            std::string funcName = py::str(value.attr("__name__"));
            info = "function '" + funcName + "'";
        } else {
            info = "type " + typeName;
        }
        std::cout << "WARNING: Overwriting key " << key << " with value " << info << "\n";
    }
    this->configDict[key] = value;
}

const py::dict& PythonConfigDict::getDict(){//Need to check if anything bad happens if dict is empty
    return this->configDict;
}

void PythonConfigDict::clearConfig(){
    this->configDict.attr("clear")();
}

std::any PythonConfigDict::getConfig(const std::string& key){
    if(this->configDict.contains(key)){
        return this->configDict[key];
    } else {
        throw std::runtime_error("No matching configuration found");
    }
}

}
