#pragma once

#include <memory>
#include <iostream>

#ifdef EXPORT_DLL
#define MY_LIB_API __declspec(dllexport)
#else
#define MY_LIB_API __declspec(dllimport)
#endif // EXPORT_DLL

class MY_LIB_API BInterface {
public:
  ~BInterface() = default;
  virtual void setInputData(int b) = 0;
  virtual void printData() = 0;

protected:
  BInterface() = default;
};

MY_LIB_API std::shared_ptr<BInterface> CreateClassB();