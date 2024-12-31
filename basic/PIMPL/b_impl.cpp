#include "b_interface.h"

class BImpl : public BInterface {

public:
  BImpl() = default;
  ~BImpl() = default;

  void setInputData(int b) override {
    b_ = b;
  }

  void printData() override {
    std::cout<<"data:"<<b_<<"\n";
  }

private:
  int b_;
};

std::shared_ptr<BInterface> CreateClassB() {
  return std::make_shared<BImpl>();
}