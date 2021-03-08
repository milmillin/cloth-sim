#pragma once

#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <Eigen/Core>

#include <atomic>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "Pipeline.h"
#include "PipelineSettings.h"

namespace clothsim {

enum LayerId {
  Mesh = 0,
  Spring,
  Max
};

class Pipeline;

class MainUI : public igl::opengl::glfw::imgui::ImGuiMenu {
 public:
  MainUI();
  ~MainUI();

  void init(igl::opengl::glfw::Viewer* _viewer) override;
  void draw_viewer_window() override;
  void draw_viewer_menu() override;
  inline bool pre_draw() override;
  inline bool post_draw() override;

  std::recursive_mutex viewerDataMutex;
  inline igl::opengl::ViewerData& GetViewerData(LayerId layerId) {
    return viewer->data(layerId);
  }

  void Invalidate();

 private:
  void Update();
  void SimulateOnce();
  void SimulateForever();
  void StopSimulation();
  void ResetSimulation();

  void Playback();
  void StopPlayback();

  inline Eigen::MatrixXd& GetMeshVertices() {
    return viewer->data(LayerId::Mesh).V;
  }
  inline Eigen::MatrixXi& GetMeshFaces() {
    return viewer->data(LayerId::Mesh).F;
  }

  // VoxelPipeline
  std::unique_ptr<Pipeline> pipeline;
  PipelineSettings settings;

  // Task
  std::deque<std::pair<std::unique_ptr<std::atomic<bool>>,
                       std::unique_ptr<std::thread>>>
      tasks;

  // Playback
  std::atomic<bool> m_isPlayingBack;
  std::atomic<int> m_currentFrame;
};

}  // namespace gripper