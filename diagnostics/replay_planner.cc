/* 녹화된 ModelState/ControlState로 LateralPlanner를 재실행한다.
 * 녹화된 인지 결과에 대해 플래너가 무엇을 요구했는지 오프라인으로 재현한다.
 * 사용: replay_planner [--laneless] <out.csv> <events.bin...> */
#include "ipc_messages.h"
#include "recorded_model_state.h"
#include "lateral_controller.h"
#include "lateral_planner.h"
#include "recording_format.h"
#include "control_params.h"
#include "vehicle_can.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>

int main(int argc, char **argv) {
  SteeringParams steering;
  DrivingParams driving;
  std::vector<const char *> positional;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--laneless") == 0) {
      driving.laneless_mode = true;
    } else if (std::strncmp(argv[i], "--", 2) == 0) {
      positional.clear();
      break;
    } else {
      positional.push_back(argv[i]);
    }
  }
  if (positional.size() < 2) {
    std::fprintf(stderr, "usage: %s [--laneless] <out.csv> <events.bin...>\n", argv[0]);
    return 2;
  }
  LateralPlanner planner(steering, driving);

  VehicleCanState vehicle{};   // 블링커/개입 없음
  std::FILE *out = std::fopen(positional[0], "w");
  if (out == nullptr) {
    std::fprintf(stderr, "cannot open %s\n", positional[0]);
    return 1;
  }
  std::fprintf(out, "t,v_kph,measured,des_rec,des_replay,target_curv,target_y,"
                    "lane_l,lane_r,prob_l,prob_r,d_prob,laneless,lane_w,mpc_valid,heading0,heading_target\n");

  float v_kph = 0.0f, measured = 0.0f, des_rec = 0.0f, prev_des = 0.0f;
  bool have_cs = false;
  for (size_t a = 1; a < positional.size(); ++a) {
    std::ifstream f(positional[a], std::ios::binary);
    EventFileHeader hdr{};
    f.read(reinterpret_cast<char *>(&hdr), sizeof(hdr));
    if (std::memcmp(hdr.magic, "K230LOG1", 8) != 0) continue;
    f.seekg(hdr.header_size);
    EventRecordHeader rh{};
    std::vector<char> buf;
    while (f.read(reinterpret_cast<char *>(&rh), sizeof(rh))) {
      buf.resize(rh.payload_size);
      if (!f.read(buf.data(), rh.payload_size)) break;
      if (rh.type == static_cast<uint16_t>(RecordType::ControlState) &&
          rh.payload_size >= sizeof(ControlState)) {
        ControlState cs{};
        std::memcpy(&cs, buf.data(), sizeof(cs));
        v_kph = cs.ego_speed_kph > 0.0f ? cs.ego_speed_kph : cs.cluster_speed_kph;
        measured = cs.actual_curvature;
        des_rec = cs.desired_curvature;
        have_cs = true;
      } else if (rh.type == static_cast<uint16_t>(RecordType::ModelState)) {
        if (!have_cs) continue;
        ModelState ms{};
        if (!decode_recorded_model_state(buf.data(), rh.payload_size, hdr.version, &ms)) continue;
        const float v = v_kph / 3.6f;
        LateralTarget t = planner.update(ms, vehicle, v, measured, true, 0.0f);
        /* 곡률 보정은 컨트롤러와 같은 100 Hz 틱으로 돌린다. 틱당 변화율 제한이
         * 있어 모델 주기로 한 번만 부르면 5배 과하게 걸린다. plan 나이는 틱마다
         * 늘어난다. */
        float des = prev_des;
        for (int tick = 0; tick < 5; ++tick) {
          des = lag_adjusted_desired_curvature(t, v, 0.01f * tick,
                                               steering.steer_actuator_delay, prev_des);
          if (t.valid) prev_des = des;
        }
        std::fprintf(out, "%.3f,%.1f,%.6f,%.6f,%.6f,%.6f,%.3f,"
                          "%.3f,%.3f,%.2f,%.2f,%.2f,%d,%.2f,%d,%.4f,%.4f\n",
                     rh.timestamp_ns * 1e-9, v_kph, measured, des_rec, des,
                     t.curvature, t.target_y_m,
                     t.lane_left_y_m, t.lane_right_y_m,
                     t.lane_left_prob, t.lane_right_prob, t.lane_d_prob,
                     t.laneless_mode ? 1 : 0, t.lane_width_m,
                     t.mpc_solution_valid ? 1 : 0, t.heading_rad, t.psis[kLateralControlN - 1]);
      }
    }
  }
  std::fclose(out);
  return 0;
}
