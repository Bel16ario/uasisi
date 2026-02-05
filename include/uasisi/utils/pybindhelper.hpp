#ifndef PYBINDHELPER_HPP
#define PYBINDHELPER_HPP

#include <any>
#include <pybind11/embed.h>
namespace py = pybind11;
//Apparently there are some problems with visibility mismatches that go way above my head so I will just tell the compiler to ignore them. Maybe later it will be worth it to fix.
namespace uasisi{

class PythonInterpreter{ //Singleton

    public:

    static PythonInterpreter& getInstance();
    static void destroyInstance();
    PythonInterpreter(const PythonInterpreter&) = delete;
    PythonInterpreter& operator=(const PythonInterpreter&) = delete;


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

    const py::dict& getDict() const;
    void clearConfig();
    std::any getConfig(const std::string& key) const;

    private:

    py::dict configDict;

};


}
#endif
