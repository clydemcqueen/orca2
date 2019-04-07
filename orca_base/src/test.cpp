#include "orca_base/model.hpp"

void test4to6()
{
  std::array<double, 16> four;
  std::array<double, 36> six;

  for (int i = 0; i < four.size(); ++i)
  {
    four[i] = i;
  }

  for (int i = 0; i < four.size(); ++i)
  {
    std::cout << "four[" << i << "]: " << four[i] << std::endl;
  }

  orca_base::OrcaOdometry::covar_4_to_6(four, six);

  for (int i = 0; i < six.size(); ++i)
  {
    std::cout << "six[" << i << "]: " << six[i] << std::endl;
  }
}

int main(int argc, char** argv)
{
  test4to6();
  return 0;
}
