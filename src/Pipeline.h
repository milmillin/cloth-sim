#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <atomic>
#include <vector>
#include <mutex>

#include "MainUI.h"
#include "PipelineSettings.h"
#include "Utils.h"

namespace clothsim {

class MainUI;

class Pipeline {
 public:
  Pipeline(MainUI* mainUI);

  void UpdateSettings(const PipelineSettings& settings, bool isInit = false);
  void Step();
  void StepForever();
  inline void ResetSimulation() { CreateMeshDependentResources(); }
  inline bool IsReady() const { return m_isReady; }
  inline bool IsRunning() const { return m_isRunning; }
  inline void Stop() { m_isRunning = false; }
  inline const Eigen::MatrixXi& GetFaces() const { return m_mesh_F; }
  inline const Eigen::MatrixXi& GetSprings() const { return m_springs; }

  mutable std::mutex historyMutex;
  inline size_t GetNumFrames() const { 
    std::lock_guard<std::mutex> guard(historyMutex);
    return m_history.size();
  }
  inline Eigen::MatrixXd GetFrame(size_t id) const {
    std::lock_guard<std::mutex> guard(historyMutex);
    return m_history[id];
  }
 private:
  void GenerateMesh(const PipelineSettings& settings);
  void GenerateGridMesh(const PipelineSettings& settings);
  void CreateMeshDependentResources();

  void StepImpl();
  void ComputeAcceleration();
  void SatisfyConstraints();
  double ComputeConstraints();

  MainUI* m_mainUI;
  std::atomic<bool> m_isReady;
  std::atomic<bool> m_isRunning;
  PipelineSettings m_settings;

  Eigen::MatrixXd m_mesh_V;
  Eigen::MatrixXi m_mesh_F;
  std::vector<int> m_boundaries;
  std::vector<std::pair<int, int>> m_constraints;
  std::vector<std::pair<int, int>> m_shears;
  std::vector<bool> m_bidirectional;
  std::vector<double> m_restLengths;
  std::vector<double> m_shearLengths;
  Eigen::MatrixXi m_springs;

  MatrixRX3d m_X;
  MatrixRX3d m_V;
  MatrixRX3d m_A;
  MatrixRX3d m_newA;

  std::vector<Eigen::MatrixXd> m_history;
};

}  // namespace clothsim
