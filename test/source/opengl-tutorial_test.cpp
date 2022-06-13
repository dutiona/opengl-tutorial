#include "lib.hpp"

auto main() -> int
{
  library lib;

  return lib.name == "opengl-tutorial" ? 0 : 1;
}
