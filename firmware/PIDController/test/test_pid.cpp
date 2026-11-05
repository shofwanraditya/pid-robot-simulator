/**
 * Desktop unit test untuk PIDController (tanpa hardware).
 *   g++ -std=c++11 -I../src ../src/PIDController.cpp test_pid.cpp -o test_pid && ./test_pid
 */
#include <cmath>
#include <cstdio>
#include "PIDController.h"

static int failures = 0;
#define CHECK(cond, msg)                                   \
  do {                                                     \
    if (cond) std::printf("  [PASS] %s\n", msg);           \
    else { std::printf("  [FAIL] %s\n", msg); failures++; } \
  } while (0)

static bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

int main() {
  std::printf("PIDController C++ tests\n");

  {
    PIDController pid(2.0f, 0.0f, 0.0f, -1000.0f, 1000.0f);
    CHECK(near(pid.compute(10.0f, 4.0f, 0.01f), 12.0f, 1e-5f), "P-only: output = Kp * error");
  }
  {
    PIDController pid(0.0f, 3.0f, 0.0f, -1000.0f, 1000.0f);
    for (int k = 0; k < 100; k++) pid.compute(1.0f, 0.0f, 0.01f);
    CHECK(near(pid.getOutput(), 3.0f, 1e-3f), "I-term integrates constant error");
  }
  {
    PIDController pid(0.0f, 0.0f, 1.0f, -1000.0f, 1000.0f);
    pid.compute(0.0f, 0.0f, 0.01f);
    CHECK(near(pid.compute(100.0f, 0.0f, 0.01f), 0.0f, 1e-6f), "No derivative kick on setpoint step");
    CHECK(near(pid.compute(100.0f, 1.0f, 0.01f), -100.0f, 1e-2f), "D = -Kd * dy/dt");
  }
  {
    PIDController pid(100.0f, 0.0f, 0.0f, -5.0f, 5.0f);
    CHECK(pid.compute(10.0f, 0.0f, 0.01f) == 5.0f && pid.isSaturated(), "Output clamped to max");
  }
  {
    PIDController aw(1.0f, 10.0f, 0.0f, -1.0f, 1.0f);
    PIDController noAw(1.0f, 10.0f, 0.0f, -1.0f, 1.0f);
    noAw.setAntiWindup(false);
    for (int k = 0; k < 500; k++) { aw.compute(10.0f, 0.0f, 0.01f); noAw.compute(10.0f, 0.0f, 0.01f); }
    CHECK(aw.getITerm() <= 1.0f + 1e-5f, "Anti-windup bounds integrator");
    CHECK(noAw.getITerm() > 400.0f, "Without anti-windup integrator winds up");
  }
  {
    // Closed loop: plant orde-1  τ dy/dt = -y + u
    PIDController pid(2.0f, 4.0f, 0.0f, -10.0f, 10.0f);
    float y = 0.0f;
    const float dt = 0.001f, tau = 0.2f;
    for (int k = 0; k < 5000; k++) {
      const float u = pid.compute(1.0f, y, dt);
      y += dt * (-y + u) / tau;
    }
    CHECK(near(y, 1.0f, 1e-3f), "Closed-loop first-order plant reaches setpoint");
  }

  std::printf(failures ? "\n%d test(s) FAILED\n" : "\nAll tests passed\n", failures);
  return failures ? 1 : 0;
}
