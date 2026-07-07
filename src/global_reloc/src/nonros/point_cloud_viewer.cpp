#include "global_reloc/nonros/point_cloud_viewer.hpp"

#include <Eigen/Geometry>

// Use ImGui's bundled OpenGL3 loader (defines gl* symbols).
#include <imgui_impl_opengl3_loader.h>

#include <cmath>
#include <algorithm>

namespace global_reloc {

namespace {
const char* kVS = R"(
#version 330 core
layout(location=0) in vec3 a_pos;
uniform mat4 u_MVP;
uniform float u_pt_size;
void main(){
  gl_Position = u_MVP * vec4(a_pos, 1.0);
  gl_PointSize = u_pt_size;
})";
const char* kFS = R"(
#version 330 core
out vec4 frag;
uniform vec3 u_color;
void main(){
  // round points (discard corners) for nicer look
  vec2 c = gl_PointCoord - vec2(0.5);
  if (dot(c,c) > 0.25) discard;
  frag = vec4(u_color, 1.0);
})";

GLuint compile(GLenum type, const char* src) {
  GLuint s = glCreateShader(type);
  glShaderSource(s, 1, &src, nullptr);
  glCompileShader(s);
  return s;
}
}  // namespace

PointCloudViewer::PointCloudViewer() = default;
PointCloudViewer::~PointCloudViewer() { shutdown(); }

void PointCloudViewer::init() {
  if (inited_) return;
  GLuint vs = compile(GL_VERTEX_SHADER, kVS);
  GLuint fs = compile(GL_FRAGMENT_SHADER, kFS);
  program_ = glCreateProgram();
  glAttachShader(program_, vs);
  glAttachShader(program_, fs);
  glLinkProgram(program_);
  glDeleteShader(vs);
  glDeleteShader(fs);

  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  inited_ = true;
}

void PointCloudViewer::shutdown() {
  if (!inited_) return;
  glDeleteProgram(program_);
  glDeleteVertexArrays(1, &vao_);
  glDeleteBuffers(1, &vbo_);
  program_ = vao_ = vbo_ = 0;
  inited_ = false;
}

void PointCloudViewer::updateView(Eigen::Matrix4f& V, Eigen::Matrix4f& P,
                                   float aspect) const {
  // Orbit camera: eye from yaw/pitch/distance around target.
  // forward = direction from eye toward target.
  float cy = std::cos(yaw_), sy = std::sin(yaw_);
  float cp = std::cos(pitch_), sp = std::sin(pitch_);
  Eigen::Vector3f fwd(sy * cp, sp, cy * cp);   // points eye -> target
  fwd.normalize();
  Eigen::Vector3f eye = target_ - fwd * distance_;
  // camera basis (OpenGL convention: -Z is forward, +Y up).
  Eigen::Vector3f s = fwd.cross(world_up_).normalized();
  Eigen::Vector3f u = s.cross(fwd).normalized();
  Eigen::Matrix4f Lv = Eigen::Matrix4f::Identity();
  // Rows: [s ; u ; -fwd], translation = -dot(axis, eye)
  Lv.block<1, 3>(0, 0) = s.transpose();
  Lv.block<1, 3>(1, 0) = u.transpose();
  Lv.block<1, 3>(2, 0) = (-fwd).transpose();
  Lv(0, 3) = -s.dot(eye);
  Lv(1, 3) = -u.dot(eye);
  Lv(2, 3) = fwd.dot(eye);
  V = Lv;

  // Perspective (OpenGL, looking down -Z).
  float fov = 45.0f * static_cast<float>(M_PI) / 180.0f;
  float nearp = 0.05f, farp = 5000.0f;
  float f = 1.0f / std::tan(fov / 2.0f);
  P.setZero();
  P(0, 0) = f / aspect;
  P(1, 1) = f;
  P(2, 2) = (farp + nearp) / (nearp - farp);
  P(2, 3) = (2 * farp * nearp) / (nearp - farp);
  P(3, 2) = -1.0f;
}

void PointCloudViewer::render(const std::vector<PointSet>& sets, int fb_w, int fb_h) {
  if (!inited_) return;
  glViewport(0, 0, fb_w, fb_h);
  glClearColor(0.12f, 0.12f, 0.13f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_PROGRAM_POINT_SIZE);

  Eigen::Matrix4f V, P;
  updateView(V, P, static_cast<float>(fb_w) / static_cast<float>(fb_h));
  Eigen::Matrix4f MVP = (P * V).transpose();  // Eigen is col-major; GL wants col-major.

  glUseProgram(program_);
  glUniformMatrix4fv(glGetUniformLocation(program_, "u_MVP"), 1, GL_FALSE, MVP.data());

  glBindVertexArray(vao_);
  glEnableVertexAttribArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  for (const auto& s : sets) {
    if (!s.visible || s.points.empty()) continue;
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(s.points.size() * sizeof(Eigen::Vector3f)),
                 s.points.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Eigen::Vector3f), nullptr);
    glUniform3fv(glGetUniformLocation(program_, "u_color"), 1, s.color);
    glUniform1f(glGetUniformLocation(program_, "u_pt_size"), s.point_size);
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(s.points.size()));
  }
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glDisableVertexAttribArray(0);
  glBindVertexArray(0);
}

void PointCloudViewer::rotate(float dx, float dy) {
  yaw_ -= dx * 0.01f;
  pitch_ = std::clamp(pitch_ - dy * 0.01f, -1.5f, 1.5f);
}
void PointCloudViewer::pan(float dx, float dy) {
  float scale = distance_ * 0.0015f;
  float cy = std::cos(yaw_), sy = std::sin(yaw_);
  float cp = std::cos(pitch_);
  Eigen::Vector3f fwd(sy * cp, std::sin(pitch_), cy * cp);
  Eigen::Vector3f right = fwd.cross(world_up_).normalized();
  Eigen::Vector3f up = right.cross(fwd).normalized();
  target_ += right * (-dx * scale) + up * (dy * scale);
}
void PointCloudViewer::zoom(float dy) {
  distance_ = std::clamp(distance_ * (1.0f + dy * 0.001f), 0.1f, 4000.0f);
}

void PointCloudViewer::fitTo(const std::vector<PointSet>& sets) {
  Eigen::Vector3f mn = Eigen::Vector3f::Constant(1e9f);
  Eigen::Vector3f mx = Eigen::Vector3f::Constant(-1e9f);
  bool any = false;
  for (const auto& s : sets) {
    if (!s.visible || s.points.empty()) continue;
    for (const auto& p : s.points) {
      any = true;
      mn = mn.cwiseMin(p);
      mx = mx.cwiseMax(p);
    }
  }
  if (!any) return;
  target_ = (mn + mx) * 0.5f;
  distance_ = (mx - mn).norm() * 1.2f + 1.0f;
  yaw_ = 0.6f; pitch_ = 0.4f;
}

}  // namespace global_reloc
