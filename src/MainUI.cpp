#include "MainUI.h"

#include <imgui.h>

namespace clothsim {

MainUI::MainUI()
    : igl::opengl::glfw::imgui::ImGuiMenu(),
      m_currentFrame(0),
      m_isPlayingBack(false) {}

MainUI::~MainUI() {
  StopSimulation();
  while (!tasks.empty()) {
    tasks.front().second->join();
    tasks.pop_front();
  }
}

void MainUI::init(igl::opengl::glfw::Viewer* _viewer) {
  igl::opengl::glfw::imgui::ImGuiMenu::init(_viewer);

  viewer->core().is_animating = true;

  // Add other layers
  for (int layerID = 1; layerID < LayerId::Max; layerID++) {
    viewer->data_list.emplace_back();
    viewer->data_list.back().id = (LayerId)layerID;
  }
  viewer->next_data_id = LayerId::Max;

  // Set default point size
  viewer->data_list[LayerId::CenterOfMass].point_size = 8;
  viewer->data_list[LayerId::AllContacts].point_size = 8;
  viewer->data_list[LayerId::FilteredContacts].point_size = 8;
  viewer->data_list[LayerId::BestContacts].point_size = 8;

  // Set default
  // viewer->data_list[LayerId::Offset].show_lines = false;
  // viewer->data_list[LayerId::GripperMesh].show_lines = false;
  viewer->data_list[LayerId::AllContacts].is_visible = false;

  viewer->core().orthographic = true;
}

void MainUI::draw_viewer_window() {
  float menu_width = 180.f * menu_scaling();
  ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);

// Work around for https://github.com/libigl/libigl/issues/1669 at the moment
#ifdef _MSC_VER
  ImGui::SetNextWindowSize(ImVec2(300.f, 600.f), ImGuiCond_FirstUseEver);
#else
  ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSizeConstraints(ImVec2(menu_width, -1.0f),
                                      ImVec2(300.0f, -1.0f));
#endif
  bool _viewer_menu_visible = true;

#ifdef _MSC_VER
  ImGui::Begin(
      "Viewer", &_viewer_menu_visible, ImGuiWindowFlags_NoSavedSettings);
#else
  ImGui::Begin(
      "Viewer",
      &_viewer_menu_visible,
      ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize);
#endif
  ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.4f);
  if (callback_draw_viewer_menu) {
    callback_draw_viewer_menu();
  } else {
    draw_viewer_menu();
  }
  ImGui::PopItemWidth();
  ImGui::End();
}

void MainUI::draw_viewer_menu() {
  float w = ImGui::GetContentRegionAvailWidth();
  float p = ImGui::GetStyle().FramePadding.x;
  if (ImGui::CollapsingHeader("Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(120.f);
    ImGui::InputDouble("Waist Radius (m)", &settings.waistRadius, 0.001, 0.01);
    ImGui::InputDouble("Length (m)", &settings.length, 0.001, 0.01);
    ImGui::InputDouble("Grid Spacing (m)", &settings.gridSpacing, 0.001, 0.01);
    ImGui::InputDouble("Timestep (s)", &settings.timeStep, 0.001, 0.01);
    ImGui::InputDouble("kShear", &settings.kShear, 0.01, 0.1);
    ImGui::PopItemWidth();

    if (pipeline == nullptr || pipeline->IsReady()) {
      if (ImGui::Button("Update Settings", ImVec2(w - p, 0))) {
        Update();
      }
      if (ImGui::Button("Reset Simulation", ImVec2(w - p, 0))) {
        ResetSimulation();
      }
      if (pipeline != nullptr) {
        if (ImGui::Button("Step", ImVec2(w - p, 0))) {
          SimulateOnce();
        }
        if (ImGui::Button("Step Forever", ImVec2(w - p, 0))) {
          SimulateForever();
        }
      }
    } else {
      if (pipeline->IsRunning()) {
        if (ImGui::Button("Stop", ImVec2(w - p, 0))) {
          StopSimulation();
        }
      } else {
        ImGui::Text("Busy...");
      }
    }
  }
  if (pipeline != nullptr) {
    if (ImGui::CollapsingHeader("Playback", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::PushID("Playback");
      ImGui::PushItemWidth(120.f);
      ImGui::Text(
          "Frame: %d of %d", m_currentFrame + 1, (int)pipeline->GetNumFrames());

      if (!m_isPlayingBack) {
        if (ImGui::Button("First")) {
          m_currentFrame = 0;
          Invalidate();
        }
        ImGui::SameLine();
        if (ImGui::Button("Prev")) {
          int old = m_currentFrame.load();
          if (old > 0) {
            m_currentFrame.compare_exchange_weak(old, old - 1);
          }
          Invalidate();
        }
        ImGui::SameLine();
        if (ImGui::Button("Play")) {
          Playback();
        }
        ImGui::SameLine();
        if (ImGui::Button("Next")) {
          int old = m_currentFrame.load();
          if (old < pipeline->GetNumFrames() - 1) {
            m_currentFrame.compare_exchange_weak(old, old + 1);
          }
          Invalidate();
        }
        ImGui::SameLine();
        if (ImGui::Button("Last")) {
          m_currentFrame = pipeline->GetNumFrames() - 1;
          Invalidate();
        }
      } else {
        if (ImGui::Button("Stop", ImVec2(w - p, 0))) {
          StopPlayback();
        }
      }

      ImGui::PopItemWidth();
      ImGui::PopID();
    }
  }
  if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushID("View");
    if (ImGui::InputFloat("Point Size",
                          &(viewer->data(LayerId::CenterOfMass).point_size),
                          1,
                          2,
                          "%.0f")) {
      viewer->data(LayerId::AllContacts).point_size =
          viewer->data(LayerId::FilteredContacts).point_size =
              viewer->data(LayerId::BestContacts).point_size =
                  viewer->data(LayerId::CenterOfMass).point_size;
    }

    ImGui::Checkbox("Mesh", (bool*)&(viewer->data(LayerId::Mesh).is_visible));
    ImGui::Checkbox("Offset Mesh",
                    (bool*)&(viewer->data(LayerId::Offset).is_visible));
    ImGui::Checkbox(
        "Gripper Direction",
        (bool*)&(viewer->data(LayerId::GripperDirection).is_visible));
    ImGui::Checkbox("Center of Mass",
                    (bool*)&(viewer->data(LayerId::CenterOfMass).is_visible));
    ImGui::Checkbox("All Contacts",
                    (bool*)&(viewer->data(LayerId::AllContacts).is_visible));
    ImGui::Checkbox(
        "Filtered Contacts",
        (bool*)&(viewer->data(LayerId::FilteredContacts).is_visible));
    ImGui::Checkbox("Best Contacts",
                    (bool*)&(viewer->data(LayerId::BestContacts).is_visible));
    ImGui::Checkbox("Gripper",
                    (bool*)&(viewer->data(LayerId::GripperMesh).is_visible));

    ImGui::PopID();
  }
}

void MainUI::Invalidate() {
  if (pipeline == nullptr) return;
  m_currentFrame = std::min((int)pipeline->GetNumFrames() - 1,
                            std::max(0, m_currentFrame.load()));

  viewerDataMutex.lock();

  igl::opengl::ViewerData& data = GetViewerData(LayerId::Mesh);
  data.clear();
  data.set_mesh(pipeline->GetFrame(m_currentFrame), pipeline->GetFaces());

  viewerDataMutex.unlock();
}

void MainUI::Update() {
  bool isInit = false;
  if (pipeline == nullptr) {
    pipeline.reset(new Pipeline(this));
    isInit = true;
  }

  // Update using new thread
  auto done = new std::atomic<bool>(false);
  auto task = new std::thread([=] {
    pipeline->UpdateSettings(settings, isInit);
    this->Invalidate();
    done->store(true);
  });
  tasks.push_back(
      std::move(std::make_pair(std::unique_ptr<std::atomic<bool>>(done),
                               std::unique_ptr<std::thread>(task))));
}

void MainUI::SimulateOnce() {
  if (pipeline == nullptr || !pipeline->IsReady()) return;
  // Update using new thread
  auto done = new std::atomic<bool>(false);
  auto task = new std::thread([=] {
    pipeline->Step();
    done->store(true);
  });
  tasks.push_back(
      std::move(std::make_pair(std::unique_ptr<std::atomic<bool>>(done),
                               std::unique_ptr<std::thread>(task))));
}

void MainUI::SimulateForever() {
  if (pipeline == nullptr || !pipeline->IsReady()) return;
  // Update using new thread
  auto done = new std::atomic<bool>(false);
  auto task = new std::thread([=] {
    pipeline->StepForever();
    done->store(true);
  });
  tasks.push_back(
      std::move(std::make_pair(std::unique_ptr<std::atomic<bool>>(done),
                               std::unique_ptr<std::thread>(task))));
}

void MainUI::StopSimulation() {
  if (pipeline != nullptr) pipeline->Stop();
}

void MainUI::ResetSimulation() {
  if (pipeline == nullptr) return;
  pipeline->ResetSimulation();
  Invalidate();
}

void MainUI::Playback() {
  if (m_isPlayingBack) return;
  m_isPlayingBack = true;
  // Update using new thread
  auto done = new std::atomic<bool>(false);
  auto task = new std::thread([this, done] {
    while (this->m_isPlayingBack &&
           this->m_currentFrame < pipeline->GetNumFrames()) {
      this->Invalidate();
      this->m_currentFrame.fetch_add(1);
      std::this_thread::sleep_for(std::chrono::microseconds(16));  // 1/60 s
    }
    this->Invalidate();
    this->m_isPlayingBack = false;
    done->store(true);
  });
  tasks.push_back(
      std::move(std::make_pair(std::unique_ptr<std::atomic<bool>>(done),
                               std::unique_ptr<std::thread>(task))));
}

void MainUI::StopPlayback() {
  m_isPlayingBack = false;
}

bool MainUI::pre_draw() {
  while (!tasks.empty() && tasks.front().first->load()) {
    tasks.front().second->join();
    tasks.pop_front();
  }
  viewerDataMutex.lock();
  ImGuiMenu::pre_draw();
  return false;
}

bool MainUI::post_draw() {
  ImGuiMenu::post_draw();
  viewerDataMutex.unlock();
  return false;
}

}  // namespace clothsim