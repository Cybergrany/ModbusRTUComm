#include <unity.h>

void run_modbus_rtu_transport_tests();
void run_modbus_rtu_platform_tests();

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  run_modbus_rtu_transport_tests();
  run_modbus_rtu_platform_tests();
  return UNITY_END();
}
