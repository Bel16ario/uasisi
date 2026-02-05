#ifndef PYBINDHELPER_HPP
#define PYBINDHELPER_HPP

#include <any>
#include <pybind11/embed.h>
namespace py = pybind11;

namespace uasisi{

class PythonInterpreter{ //Singleton

    public:

    static PythonInterpreter& getInstance();
    static void destroyInstance();
    PythonInterpreter(const PythonInterpreter&) = delete("Copy construction disabled");
    PythonInterpreter& operator=(const PythonInterpreter&) = delete("Copy assignment disabled");


    private:
    
    inline static PythonInterpreter* instance = nullptr;
    PythonInterpreter() : guard() {}
    ~PythonInterpreter() = default;

    py::scoped_interpreter guard;

};

class PythonConfigDict{
    public:

    void setConfig(const std::string& key, const std::any& value);
    void setConfigInt(const std::string& key, const int value);
    void setConfigBool(const std::string& key, const bool value);
    void setConfigDouble(const std::string& key, const double value);
    void setConfigString(const std::string& key, const std::string& value);
    void setConfigObject(const std::string& key, const py::object& value);

    const py::dict& getDict();
    void clearConfig();
    std::any getConfig(const std::string& key);

    private:

    py::dict configDict;

};


}
#endif
