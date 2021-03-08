#pragma once

namespace clothsim {

struct PipelineSettings {
  double waistRadius = 0.1;
  double length = 0.75;

  double gridSpacing = 0.038;

  double timeStep = 0.01;
  double kShear = 250;
  double kDamping = 0.5;
};

}  // namespace clothsim