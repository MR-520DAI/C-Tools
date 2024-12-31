#include "b_interface.h"

int main() {
  std::shared_ptr<BInterface> B = CreateClassB();

  B->setInputData(5);
  B->printData();

  return 0;
}