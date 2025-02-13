// Summary:
//      This test cycles the proportional solenoid between two positions
//
// How to run:
//      deploy_test.sh <device_id> psol <min> <max> <step>
//
// Automation:
//      TBD - which python script to run?
//

#include "hal.h"

// test parameters
static constexpr Duration Delay{milliseconds(10)};
static constexpr float PSolMin{TEST_PARAM_1};
static constexpr float PSolMax{TEST_PARAM_2};
static constexpr float InitialStep{TEST_PARAM_3};

void RunTest() {
  hal.Init();

  float psol_position = PSolMin;
  float step = InitialStep;

  while (true) {
    hal.PSolValue(psol_position);
    hal.Delay(Delay);

    hal.WatchdogHandler();

    psol_position += step;
    if (psol_position >= PSolMax) {
      // switch to ramp-down state
      psol_position = PSolMax;
      step = -step;
    } else if (psol_position <= PSolMin) {
      // switch to ramp-up state
      psol_position = PSolMin;
      step = -step;
    }
  }
}
