// Simple GUI for global_reloc: load/build a map, load/generate a query cloud,
// run relocalization, and visualize map/query/aligned clouds + the estimated pose.
//
// Build: see CMakeLists reloc_gui target. Requires glfw3 + ImGui + gl3w.
#include "global_reloc/nonros/reloc_facade.hpp"
#include "global_reloc/nonros/point_cloud_viewer.hpp"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>
#include <cstring>
#include <cstdio>

using namespace global_reloc;

struct AppState {
  RelocFacade facade;
  PointCloudViewer viewer;
  std::vector<PointSet> sets;   // [0]=map [1]=query_raw [2]=query_aligned
  RelocOutcome last;
  // synthetic demo state
  int synth_density = 4000;
  float synth_radius = 4.0f;
  float perturb_t[3] = {0.3f, -0.2f, 0.0f};
  float perturb_yaw = 0.15f;
  bool show_aligned = true;
  bool show_query = true;
  // file paths buffer
  char pcd_path[512] = "";
  char gkey_path[512] = "";
  char query_path[512] = "";
  char params_path[512] = "config/params.yaml";
  char status_msg[256] = "ready";
  bool running = true;
  // mouse state for camera drag
  bool ml = false, mr = false;
  double lastx = 0, lasty = 0;
};

static void glfwError(int, const char* desc) {
  std::cerr << "GLFW error: " << desc << "\n";
}

static void fillSets(AppState& s) {
  s.sets.clear();
  PointSet ms; ms.color[0] = 0.6f; ms.color[1] = 0.6f; ms.color[2] = 0.62f;
  ms.point_size = 1.5f; ms.points = s.last.map_points; ms.visible = true;
  s.sets.push_back(ms);

  PointSet qs; qs.color[0] = 0.95f; qs.color[1] = 0.35f; qs.color[2] = 0.35f;
  qs.point_size = 3.0f; qs.points = s.last.query_raw; qs.visible = s.show_query;
  s.sets.push_back(qs);

  PointSet as; as.color[0] = 0.35f; as.color[1] = 0.95f; as.color[2] = 0.45f;
  as.point_size = 3.0f; as.points = s.last.query_aligned; as.visible = s.show_aligned;
  s.sets.push_back(as);
}

static void runSynthetic(AppState& s) {
  pcl::PointCloud<pcl::PointXYZ> room = makeSyntheticRoom(s.synth_density);
  if (!s.facade.hasMap()) {
    // build map from the same room
    char tmp[] = "tmp_scene_XXXXXX.pcd";
    pcl::io::savePCDFileBinary(tmp, room);
    if (!s.facade.buildMapFromPCD(tmp, "", "scene")) {
      std::snprintf(s.status_msg, sizeof(s.status_msg), "build map failed");
      std::remove(tmp);
      return;
    }
    std::remove(tmp);
    std::snprintf(s.status_msg, sizeof(s.status_msg), "synthetic map built (%zu pts)",
                  s.facade.mapSize());
  }
  Eigen::Isometry3d perturb = Eigen::Isometry3d::Identity();
  perturb.translation() = Eigen::Vector3d(s.perturb_t[0], s.perturb_t[1], s.perturb_t[2]);
  perturb.rotate(Eigen::AngleAxisd(s.perturb_yaw, Eigen::Vector3d::UnitZ()));
  SyntheticQuery sq = makeSyntheticQuery(room, Eigen::Vector3d(2.0, 2.0, 1.0),
                                         s.synth_radius, perturb);
  s.last = s.facade.estimate(sq.cloud);
  fillSets(s);
  s.viewer.fitTo(s.sets);
  const RelocResult& r = s.last.result;
  if (r.converged) {
    std::snprintf(s.status_msg, sizeof(s.status_msg),
                  "RELOC OK  score=%.3f  cand=%d", r.score, r.candidates_evaluated);
  } else {
    std::snprintf(s.status_msg, sizeof(s.status_msg), "RELOC FAIL  %s",
                  r.failure_reason.c_str());
  }
}

int main(int argc, char** argv) {
  glfwSetErrorCallback(glfwError);
  if (!glfwInit()) return 1;
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

  GLFWwindow* win = glfwCreateWindow(1280, 800, "global_reloc demo", nullptr, nullptr);
  if (!win) { glfwTerminate(); return 1; }
  glfwMakeContextCurrent(win);
  glfwSwapInterval(1);

  AppState s;
  s.viewer.init();

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui_ImplGlfw_InitForOpenGL(win, true);
  ImGui_ImplOpenGL3_Init("#version 330");

  glfwSetWindowUserPointer(win, &s);
  glfwSetMouseButtonCallback(win, [](GLFWwindow* w, int b, int a, int){
    auto* st = static_cast<AppState*>(glfwGetWindowUserPointer(w));
    if (b == GLFW_MOUSE_BUTTON_LEFT) st->ml = (a == GLFW_PRESS);
    if (b == GLFW_MOUSE_BUTTON_RIGHT) st->mr = (a == GLFW_PRESS);
  });
  glfwSetScrollCallback(win, [](GLFWwindow* w, double, double yoff){
    auto* st = static_cast<AppState*>(glfwGetWindowUserPointer(w));
    st->viewer.zoom(static_cast<float>(-yoff));
  });

  while (!glfwWindowShouldClose(win) && s.running) {
    glfwPollEvents();
    int fb_w, fb_h; glfwGetFramebufferSize(win, &fb_w, &fb_h);

    // camera drag
    double mx, my; glfwGetCursorPos(win, &mx, &my);
    double dx = mx - s.lastx, dy = my - s.lasty;
    s.lastx = mx; s.lasty = my;
    ImGuiIO& io = ImGui::GetIO();
    bool imgui_wants = io.WantCaptureMouse;
    if (!imgui_wants) {
      if (s.ml) s.viewer.rotate(static_cast<float>(dx), static_cast<float>(dy));
      if (s.mr) s.viewer.pan(static_cast<float>(dx), static_cast<float>(dy));
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(360, 0), ImGuiCond_Once);
    ImGui::Begin("Controls", &s.running, ImGuiWindowFlags_NoMove);
    ImGui::TextDisabled("status: %s", s.status_msg);
    ImGui::Separator();

    if (ImGui::CollapsingHeader("Map", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::Text("map pts: %zu", s.facade.mapSize());
      ImGui::InputText("pcd path", s.pcd_path, sizeof(s.pcd_path));
      if (ImGui::Button("build from PCD")) {
        if (s.facade.buildMapFromPCD(s.pcd_path, "", "scene"))
          std::snprintf(s.status_msg, sizeof(s.status_msg), "map built (%zu pts)", s.facade.mapSize());
        else std::snprintf(s.status_msg, sizeof(s.status_msg), "build failed");
      }
      ImGui::InputText("gkey path", s.gkey_path, sizeof(s.gkey_path));
      if (ImGui::Button("load .gkey")) {
        if (s.facade.loadMap(s.gkey_path))
          std::snprintf(s.status_msg, sizeof(s.status_msg), "map loaded (%zu pts)", s.facade.mapSize());
        else std::snprintf(s.status_msg, sizeof(s.status_msg), "load failed");
      }
    }

    if (ImGui::CollapsingHeader("Query", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::InputText("query pcd", s.query_path, sizeof(s.query_path));
      if (ImGui::Button("run on PCD") && s.facade.hasMap()) {
        pcl::PointCloud<pcl::PointXYZ> q;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(s.query_path, q) >= 0) {
          s.last = s.facade.estimate(q);
          fillSets(s); s.viewer.fitTo(s.sets);
          const auto& r = s.last.result;
          if (r.converged) std::snprintf(s.status_msg, sizeof(s.status_msg), "OK score=%.3f", r.score);
          else std::snprintf(s.status_msg, sizeof(s.status_msg), "FAIL %s", r.failure_reason.c_str());
        } else std::snprintf(s.status_msg, sizeof(s.status_msg), "query load failed");
      }
      ImGui::Separator();
      ImGui::Text("synthetic demo");
      ImGui::SliderInt("density", &s.synth_density, 500, 20000);
      ImGui::SliderFloat("scan radius", (float*)&s.synth_radius, 1.0f, 10.0f);
      ImGui::InputFloat3("perturb t", (float*)s.perturb_t);
      ImGui::SliderFloat("perturb yaw", (float*)&s.perturb_yaw, -1.0f, 1.0f);
      if (ImGui::Button("run synthetic")) runSynthetic(s);
    }

    if (ImGui::CollapsingHeader("View")) {
      if (!s.sets.empty()) ImGui::Checkbox("show map (gray)", &s.sets[0].visible);
      if (s.sets.size() >= 2) ImGui::Checkbox("show query (red)", &s.show_query);
      if (s.sets.size() >= 3) ImGui::Checkbox("show aligned (green)", &s.show_aligned);
      if (ImGui::Button("fit view")) s.viewer.fitTo(s.sets);
    }

    if (s.last.result.converged || !s.last.query_aligned.empty()) {
      ImGui::Separator();
      ImGui::Text("Result");
      const auto& r = s.last.result;
      ImGui::Text("converged: %s  score: %.3f", r.converged ? "yes" : "no", r.score);
      const Eigen::Vector3d t = r.pose.translation();
      Eigen::Quaterniond q(r.pose.rotation());
      ImGui::Text("t = [%.2f %.2f %.2f]", t.x(), t.y(), t.z());
      ImGui::Text("q = [%.3f %.3f %.3f %.3f]", q.x(), q.y(), q.z(), q.w());
    }
    ImGui::End();

    // sync visibility flags back to sets
    if (s.sets.size() >= 2) s.sets[1].visible = s.show_query;
    if (s.sets.size() >= 3) s.sets[2].visible = s.show_aligned;

    // render 3D
    s.viewer.render(s.sets, fb_w, fb_h);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(win);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  s.viewer.shutdown();
  glfwDestroyWindow(win);
  glfwTerminate();
  return 0;
}
