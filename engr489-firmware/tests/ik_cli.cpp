// Host-only bridge for testing kinematics.cpp against the Python reference.
// Reads "x y z" per line from stdin, writes "t1 t2 t3" or "UNREACHABLE" per
// line to stdout. See tests/test_ik_parity.py.
#include "../delta_servo/kinematics.h"
#include <cstdio>

int main() {
  float xyz[3];
  float theta[3];
  while (scanf("%f %f %f", &xyz[0], &xyz[1], &xyz[2]) == 3) {
    if (ik(xyz, theta)) {
      printf("%.6f %.6f %.6f\n", theta[0], theta[1], theta[2]);
    } else {
      printf("UNREACHABLE\n");
    }
  }
  return 0;
}
