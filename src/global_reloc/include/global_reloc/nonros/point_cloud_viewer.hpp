#pragma once

// Minimal OpenGL point cloud viewer (no framework). Manages a turntable
// camera (yaw/pitch/distance/target) and renders up to a few colored point
// sets via VBOs. Designed to be driven by an ImGui main loop.

#include <Eigen/Core>
#include <vector>
#include <string>

namespace global_reloc {

struct PointSet {
  std::vector<Eigen::Vector3f> points;
  float color[3] = {0.7f, 0.7f, 0.7f};
  float point_size = 2.0f;
  bool visible = true;
};

class PointCloudViewer {
 public:
  PointCloudViewer();
  ~PointCloudViewer();

  // Call once after GL context is current.
  void init();
  void shutdown();

  // Render with the given framebuffer size. point_sets indexed by caller.
  void render(const std::vector<PointSet>& sets, int fb_w, int fb_h);

  // ---- camera control (call from input callbacks) ----
  void rotate(float dx, float dy);
  void pan(float dx, float dy);
  void zoom(float dy);
  void fitTo(const std::vector<PointSet>& sets);

 private:
  unsigned int program_ = 0;
  unsigned int vao_ = 0;
  unsigned int vbo_ = 0;
  bool inited_ = false;

  // camera state
  float yaw_ = 0.6f, pitch_ = 0.4f;
  float distance_ = 20.0f;
  Eigen::Vector3f target_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f world_up_ = Eigen::Vector3f::UnitZ();

  void updateView(Eigen::Matrix4f& V, Eigen::Matrix4f& P, float aspect) const;
};

}  // namespace global_reloc
