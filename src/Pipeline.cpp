#include "Pipeline.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>

namespace clothsim {

Pipeline::Pipeline(MainUI* mainUI)
    : m_mainUI(mainUI), m_isReady(false), m_isRunning(false), m_history(1) {}

void Pipeline::UpdateSettings(const PipelineSettings& settings, bool isInit) {
  m_isReady = false;

  if (isInit || settings.waistRadius != m_settings.waistRadius ||
      settings.length != m_settings.length ||
      settings.gridSpacing != m_settings.gridSpacing) {
    GenerateGridMesh(settings);
    CreateMeshDependentResources();
  }

  m_settings = settings;

  m_isReady = true;
}

void Pipeline::GenerateMesh(const PipelineSettings& settings) {
  double meanRadius = settings.waistRadius + settings.length / 2;
  size_t nAngleDiv =
      (size_t)ceil(meanRadius * EIGEN_PI * 2 / settings.gridSpacing);
  size_t nLateralDiv = (size_t)ceil(settings.length / settings.gridSpacing);

  double angleStep = 2 * EIGEN_PI / nAngleDiv;
  double lateralStep = settings.length / nLateralDiv;

  size_t nVertexInAngle = nLateralDiv + 1;
  size_t nFaces = nLateralDiv * nAngleDiv * 2ull;

  m_mesh_V.resize(nVertexInAngle * nAngleDiv, 3);
  m_mesh_F.resize(nFaces, 3);
  m_boundaries.clear();
  m_constraints.clear();

  for (size_t i = 0; i < nAngleDiv; i++) {
    double curAngle = angleStep * i;
    double curLateral;
    int nexti = (i + 1) % nAngleDiv;
    for (size_t j = 0; j < nLateralDiv; j++) {
      curLateral = settings.waistRadius + j * lateralStep;
      m_mesh_V.row(nVertexInAngle * i + j) = Eigen::RowVector3d(
          curLateral * cos(curAngle), 0, curLateral * sin(curAngle));

      m_mesh_F.row(nLateralDiv * 2 * i + j * 2) =
          Eigen::RowVector3i(nVertexInAngle * i + j,
                             nVertexInAngle * nexti + j,
                             nVertexInAngle * i + j + 1);
      m_mesh_F.row(nLateralDiv * 2 * i + j * 2 + 1) =
          Eigen::RowVector3i(nVertexInAngle * nexti + j,
                             nVertexInAngle * nexti + j + 1,
                             nVertexInAngle * i + j + 1);

      m_constraints.push_back(
          {nVertexInAngle * i + j, nVertexInAngle * nexti + j});
      m_constraints.push_back(
          {nVertexInAngle * i + j, nVertexInAngle * i + j + 1});
    }
    curLateral = settings.waistRadius + settings.length;
    m_mesh_V.row(nVertexInAngle * i + nLateralDiv) = Eigen::RowVector3d(
        curLateral * cos(curAngle), 0, curLateral * sin(curAngle));
    m_constraints.push_back({nVertexInAngle * i + nLateralDiv,
                             nVertexInAngle * nexti + nLateralDiv});
    m_boundaries.push_back(nVertexInAngle * i);
  }
}

void Pipeline::GenerateGridMesh(const PipelineSettings& settings) {
  double R = settings.length + settings.waistRadius;
  double r = settings.waistRadius;
  double width = 2 * R;
  size_t n = (size_t)ceil(width / settings.gridSpacing);
  double step = width / n;
  double origin = -R;
  double R2 = R * R;
  double r2 = r * r;

  std::cout << "n: " << n << "\n";

  int vCount = 0;
  std::vector<Eigen::RowVector3d> V;
  std::vector<Eigen::RowVector3i> F;

  std::vector<int> xInOut[4];
  std::vector<int> zInOut[4];

  std::vector<std::vector<int>> index(n + 1, std::vector<int>(n + 1, -1));

  for (int i = 0; i < 4; i++) {
    xInOut[i].resize(n + 1, -1);
    zInOut[i].resize(n + 1, -1);
  }

  auto AddVertex = [&V, &vCount](double x, double z) -> int {
    V.push_back(Eigen::RowVector3d(x, 0, z));
    return vCount++;
  };

  auto AddOrGetVertex = [&AddVertex, &index](
                            int xi, int zi, double x, double z) -> int {
    if (index[xi][zi] != -1) return index[xi][zi];
    // std::cout << "AddVertex: " << xi << "," << zi << "//" << x << ", " << z
    // << "\n";
    return index[xi][zi] = AddVertex(x, z);
  };

  auto PreAdd = [&AddOrGetVertex, R2, r2](
                    int xi, int zi, double x, double z) -> int {
    double cur_r2 = x * x + z * z;
    if (R2 >= cur_r2 && cur_r2 >= r2) return AddOrGetVertex(xi, zi, x, z);
    return -1;
  };

  // Get vertex between (xi, zi) (xi, zi + 1)
  auto GetBetweenX = [&](int xi, int zi) -> int {
    double curZ = origin + zi * step;
    for (int i = 0; i < 4; i++) {
      int id = xInOut[i][xi];
      if (id == -1) continue;
      if (curZ <= V[id].z() && V[id].z() <= curZ + step) {
        return id;
      }
    }
    return -1;
  };

  // Get vertex between (xi, zi) (xi + 1, zi)
  auto GetBetweenZ = [&](int xi, int zi) -> int {
    double curX = origin + xi * step;
    for (int i = 0; i < 4; i++) {
      int id = zInOut[i][zi];
      if (id == -1) continue;
      if (curX <= V[id].x() && V[id].x() <= curX + step) {
        return id;
      }
    }
    return -1;
  };

  m_boundaries.clear();
  m_constraints.clear();
  m_shears.clear();

  for (int xi = 0; xi <= n; xi++) {
    double curX = origin + xi * step;
    double det = R * R - curX * curX;
    if (det > 0) {
      double offsetZ = sqrt(det);
      xInOut[0][xi] = AddVertex(curX, -offsetZ);
      zInOut[0][xi] = AddVertex(-offsetZ, curX);
      if (offsetZ > 1e-9) {
        xInOut[3][xi] = AddVertex(curX, offsetZ);
        zInOut[3][xi] = AddVertex(offsetZ, curX);
      }

      double detr = r * r - curX * curX;
      if (detr > 0) {
        double offsetZr = sqrt(detr);
        m_boundaries.push_back(xInOut[1][xi] = AddVertex(curX, -offsetZr));
        m_boundaries.push_back(zInOut[1][xi] = AddVertex(-offsetZr, curX));
        if (offsetZr > 1e-9) {
          m_boundaries.push_back(xInOut[2][xi] = AddVertex(curX, offsetZr));
          m_boundaries.push_back(zInOut[2][xi] = AddVertex(offsetZr, curX));
        }
      }
    }
  }

  for (int xi = 0; xi < n; xi++) {
    double curX = origin + xi * step;
    double nextX = origin + (xi + 1) * step;
    for (int zi = 0; zi < n; zi++) {
      double curZ = origin + zi * step;
      double nextZ = origin + (zi + 1) * step;

      int lb = PreAdd(xi, zi, curX, curZ);
      int rb = PreAdd(xi + 1, zi, nextX, curZ);
      int lt = PreAdd(xi, zi + 1, curX, nextZ);
      int rt = PreAdd(xi + 1, zi + 1, nextX, nextZ);

      // std::cout << "block: " << lb << " " << rb << " " << lt << " " << rt
      // << "\n";

      if (lb == -1 && rb == -1 && lt == -1 && rt == -1) continue;

      int l = GetBetweenX(xi, zi);
      int r = GetBetweenX(xi + 1, zi);
      int b = GetBetweenZ(xi, zi);
      int t = GetBetweenZ(xi, zi + 1);

      std::vector<int> cycle;

      int all[8] = {lb, l, lt, t, rt, r, rb, b};

      for (int i = 0; i < 8; i++) {
        if (all[i] != -1 && (cycle.empty() || cycle.back() != all[i]))
          cycle.push_back(all[i]);
      }
      if ((xi + zi) % 2 == 0) {
        cycle.push_back(cycle.front());
        cycle.erase(cycle.begin());
      }
      for (size_t i = 1; i < cycle.size() - 1; i++) {
        F.push_back(Eigen::RowVector3i(cycle[0], cycle[i], cycle[i + 1]));
      }
      // for (size_t i = 0; i < cycle.size() - 2; i++) {
        // m_shears.push_back({cycle[i], cycle[i + 2]});
      // }
      if (cycle.size() >= 3) {
        // Structural
        for (size_t i = 0; i < cycle.size(); i++) {
          m_constraints.push_back(
              std::minmax(cycle[i], cycle[(i + 1) % cycle.size()]));
        }
        // Shear
        for (size_t i = 0; i < cycle.size() - 2; i++) {
          m_constraints.push_back(
              std::minmax(cycle[i], cycle[i + 2]));
        }
      }
    }

    // Bending
    for (int xi = 0; xi < n; xi++) {
      for (int zi = 0; zi < n - 2; zi++) {
        int u = index[xi][zi];
        int v = index[xi][zi + 2];
        if (u != -1 && v != -1) {
          m_constraints.push_back(std::minmax(u, v));
        }
        int u2 = index[zi][xi];
        int v2 = index[zi + 2][xi];
        if (u2 != -1 && v2 != -1) {
          m_constraints.push_back(std::minmax(u2, v2));        
        }
      }
    }
  }

  // consolidate constraints
  std::sort(m_constraints.begin(), m_constraints.end());
  m_constraints.resize(std::unique(m_constraints.begin(), m_constraints.end()) -
                       m_constraints.begin());

  // convert to matrix
  m_mesh_V.resize(V.size(), 3);
  m_mesh_F.resize(F.size(), 3);

  for (size_t i = 0; i < V.size(); i++) {
    m_mesh_V.row(i) = V[i];
  }
  for (size_t i = 0; i < F.size(); i++) {
    m_mesh_F.row(i) = F[i];
  }
}

void Pipeline::CreateMeshDependentResources() {
  // Compute Constraint Order
  std::vector<std::vector<int>> pth(m_mesh_V.rows());
  std::vector<int> dis(m_mesh_V.rows(), m_mesh_V.rows() + 1);
  std::queue<int> q;
  for (const auto& c : m_constraints) {
    pth[c.first].push_back(c.second);
    pth[c.second].push_back(c.first);
  }
  for (int b : m_boundaries) {
    q.push(b);
    dis[b] = 0;
  }
  while (!q.empty()) {
    int u = q.front();
    q.pop();
    for (int v : pth[u]) {
      if (dis[v] <= dis[u] + 1) continue;
      dis[v] = dis[u] + 1;
      q.push(v);
    }
  }
  for (auto& c : m_constraints) {
    if (dis[c.first] > dis[c.second]) {
      c = {c.second, c.first};
    }
  }
  std::sort(
      m_constraints.begin(),
      m_constraints.end(),
      [&](const std::pair<int, int>& a, const std::pair<int, int>& b) -> bool {
        return std::make_pair(dis[a.first], dis[a.second]) <
               std::make_pair(dis[b.first], dis[b.second]);
      });

  m_restLengths.resize(m_constraints.size());
  m_bidirectional.resize(m_constraints.size());
  m_springs.resize(m_constraints.size(), 2);
  for (size_t i = 0; i < m_constraints.size(); i++) {
    int u = m_constraints[i].first;
    int v = m_constraints[i].second;
    m_bidirectional[i] = dis[u] == dis[v];
    m_restLengths[i] = (m_mesh_V.row(u) - m_mesh_V.row(v)).norm();
    m_springs(i, 0) = u;
    m_springs(i, 1) = v;
  }

  m_shearLengths.resize(m_shears.size());
  for (size_t i = 0; i < m_shears.size(); i++) {
    int u = m_shears[i].first;
    int v = m_shears[i].second;
    m_shearLengths[i] = (m_mesh_V.row(u) - m_mesh_V.row(v)).norm();
  }

  // Initialize
  m_X = m_mesh_V;
  m_V = MatrixRX3d::Zero(m_mesh_V.rows(), 3);
  m_A = MatrixRX3d::Zero(m_mesh_V.rows(), 3);

  ComputeAcceleration();
  std::swap(m_A, m_newA);

  {
    std::lock_guard<std::mutex> guard(historyMutex);
    m_history.clear();
    m_history.push_back(m_X);
  }
}

void Pipeline::Step() {
  m_isReady = false;
  StepImpl();
  m_isReady = true;
}

void Pipeline::StepForever() {
  m_isRunning = true;
  m_isReady = false;
  while (m_isRunning) {
    StepImpl();
  }
  m_isReady = true;
}

void Pipeline::StepImpl() {
  double dt = m_settings.timeStep;
  ComputeAcceleration();

  // Verlet Integration
  m_X += m_V * dt + m_A * (dt * dt / 2);
  m_V += (m_A + m_newA) * (dt / 2);
  std::swap(m_A, m_newA);

  // Constraint
  SatisfyConstraints();

  {
    std::lock_guard<std::mutex> guard(historyMutex);
    m_history.push_back(m_X);
  }
}

void Pipeline::ComputeAcceleration() {
  m_newA = Eigen::RowVector3d(0, -9.8, 0).replicate(m_X.rows(), 1);

  // Shear force
  for (size_t i = 0; i < m_constraints.size(); i++) {
    int u = m_constraints[i].first;
    int v = m_constraints[i].second;
    Eigen::RowVector3d diff = m_X.row(u) - m_X.row(v);
    double norm = diff.norm();
    if (norm < 1e-6) continue;
    double offset = 0.5 * m_settings.kShear * (norm - m_restLengths[i]) / norm;
    diff *= offset;
    m_newA.row(v) += diff;
    m_newA.row(u) -= diff;
  }

  // Damping
  m_newA -= m_settings.kDamping * m_V;

  // Filter Boundary
#pragma omp parallel for
  for (int64_t i = 0; i < m_boundaries.size(); i++) {
    m_newA.row(m_boundaries[i]).setZero();
  }
}

void Pipeline::SatisfyConstraints() {
  int64_t nConstraints = m_constraints.size();
  int nIter = 0;
  double curConstraint;
  while ((curConstraint = ComputeConstraints()) > 1e-4 && nIter++ < 20) {
    for (int64_t i = 0; i < nConstraints; i++) {
      int u = m_constraints[i].first;
      int v = m_constraints[i].second;
      Eigen::RowVector3d diff = m_X.row(u) - m_X.row(v);
      double norm = diff.norm();
      double offset = (norm - m_restLengths[i]) / norm;
      if (norm < 1e-6) continue;
      if (m_bidirectional[i]) {
        diff *= offset / 2;
        m_X.row(v) += diff;
        m_X.row(u) -= diff;
      } else {
        diff *= offset;
        m_X.row(v) += diff;
      }
    }
  }

  std::cout << "Constraint: " << curConstraint << " nIter: " << nIter << "\n";
}

double Pipeline::ComputeConstraints() {
  int64_t nConstraints = m_constraints.size();
  double constraint = 0;

#pragma omp parallel for reduction(+ : constraint)
  for (int64_t i = 0; i < nConstraints; i++) {
    int u = m_constraints[i].first;
    int v = m_constraints[i].second;
    double curLength = (m_X.row(u) - m_X.row(v)).norm();
    if (isnan(curLength)) {
      std::cout << "NaN: " << u << ", " << v << ", " << m_restLengths[i] << "\n";    
    }
    constraint += abs(curLength - m_restLengths[i]);
  }
  return constraint;
}

}  // namespace clothsim