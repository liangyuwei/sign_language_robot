#include "tmp_torch.h"


void test_torch() {
    torch::Tensor tensor = torch::rand({2, 3});
    std::cout << tensor << std::endl;
}
