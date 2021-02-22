#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <iostream>

#include "MainUI.h"

using namespace clothsim;

int main(int argc, char* argv[]) {
  std::cout << "Num Threads: " << Eigen::nbThreads() << std::endl;

  igl::opengl::glfw::Viewer viewer;

  MainUI mainUI;
  viewer.plugins.push_back(&mainUI);
  viewer.launch(true, false, "Cloth Simulation");
}
