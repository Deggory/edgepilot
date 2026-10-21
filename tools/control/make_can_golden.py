#!/usr/bin/env python3
"""보내는 CAN 바이트와 토크 제한의 골든 값을 opendbc(상류)로 만든다. tests/hyundai_can_golden.h에 쓰고,
tests/gtest_car.cc가 edgepilot의 hyundai_can과 대조한다(체크섬, 비트 자리, 배율, 제한기 식).

  사용: tools/control/make_can_golden.py <opendbc 저장소(opendbc_repo)> > tests/hyundai_can_golden.h
        clang-format -i tests/hyundai_can_golden.h

- 바이트는 opendbc CANPacker와 opendbc의 Hyundai DBC(dbc/generator/hyundai/hyundai_can.dbc)로 만든다. 패커는 차종별
  체크섬 함수의 이름만 opendbc.car에서 가져오므로, capnp 없이 돌게 opendbc.car를 빈 모듈로 바꿔 둔다.
- LKAS11: 씨앗(카메라가 보낸 LKAS11 값)에 명령을 얹는 상류 create_lkas11의 기본 분기다(SysWarning 3, 체크섬은 앞
  6바이트와 마지막 바이트의 합).
- CLU11: 상류 create_clu11(버튼, AliveCnt1 = frame % 16).
- MDPS12: openpilot 0.8 커뮤니티 create_mdps12(ToiActive 0, ToiUnavail 1, MsgCount2 = frame % 256, 바이트 합 체크섬).
- 토크 제한: opendbc apply_driver_steer_torque_limits, K7 값(params/steering.json 기본값).
"""
import argparse
import importlib.abc
import importlib.machinery
import importlib.util
import logging
import random
import subprocess
import sys
import types
from pathlib import Path

LKAS11_FIELDS = ("CF_Lkas_LdwsActivemode", "CF_Lkas_LdwsSysState", "CF_Lkas_SysWarning", "CF_Lkas_LdwsLHWarning",
                 "CF_Lkas_LdwsRHWarning", "CF_Lkas_HbaLamp", "CF_Lkas_FcwBasReq", "CR_Lkas_StrToqReq", "CF_Lkas_ActToi",
                 "CF_Lkas_ToiFlt", "CF_Lkas_HbaSysState", "CF_Lkas_FcwOpt", "CF_Lkas_HbaOpt", "CF_Lkas_MsgCount",
                 "CF_Lkas_FcwSysState", "CF_Lkas_FcwCollisionWarning", "CF_Lkas_FusionState", "CF_Lkas_Unknown1",
                 "CF_Lkas_FcwOpt_USM", "CF_Lkas_LdwsOpt_USM", "CF_Lkas_Unknown2")
# src/car/hyundai_can.h의 토크 한계 상수(kHyundaiSteerMax, DeltaUp, DeltaDown, DriverAllowance, DriverMultiplier, DriverFactor)
K7_LIMITS = dict(STEER_MAX=384, STEER_DELTA_UP=3, STEER_DELTA_DOWN=7, STEER_DRIVER_ALLOWANCE=50,
                 STEER_DRIVER_MULTIPLIER=2, STEER_DRIVER_FACTOR=1)


class _Stub(types.ModuleType):
    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        return lambda *args, **kwargs: 0


class _CarStubs(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    """opendbc.car.* 대신 빈 모듈(패커와 lateral.py가 가져오는 이름만 있으면 된다)."""

    def find_spec(self, name, path, target=None):
        if name == "opendbc.car" or name.startswith("opendbc.car."):
            return importlib.machinery.ModuleSpec(name, self, is_package=True)
        return None

    def create_module(self, spec):
        module = _Stub(spec.name)
        module.__path__ = []
        if spec.name == "opendbc.car":
            module.ACCELERATION_DUE_TO_GRAVITY = 9.81
            module.DT_CTRL = 0.01
        if spec.name == "opendbc.car.carlog":
            module.carlog = logging.getLogger("carlog")
        return module

    def exec_module(self, module):
        pass


def load_opendbc(repo: Path):
    sys.path.insert(0, str(repo))
    sys.meta_path.insert(0, _CarStubs())
    from opendbc.can.packer import CANPacker
    spec = importlib.util.spec_from_file_location("upstream_lateral", repo / "opendbc" / "car" / "lateral.py")
    lateral = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(lateral)
    return CANPacker(str(repo / "opendbc" / "dbc" / "generator" / "hyundai" / "hyundai_can.dbc")), lateral


def pack(packer, name: str, values: dict) -> bytes:
    return bytes(packer.make_can_msg(name, 0, values)[1])


def lkas11_cases(packer, rng: random.Random):
    seeds = [  # 카메라 LKAS11 값: 꺼짐, 켜짐 표시, 비트가 많이 선 것
        dict.fromkeys(LKAS11_FIELDS, 0) | {"CR_Lkas_StrToqReq": 0, "CF_Lkas_LdwsSysState": 1,
                                           "CF_Lkas_FcwOpt_USM": 1, "CF_Lkas_LdwsOpt_USM": 2},
        dict.fromkeys(LKAS11_FIELDS, 0) | {"CR_Lkas_StrToqReq": 0, "CF_Lkas_LdwsSysState": 3, "CF_Lkas_FcwSysState": 3,
                                           "CF_Lkas_FcwOpt_USM": 2, "CF_Lkas_LdwsOpt_USM": 2, "CF_Lkas_HbaOpt": 1},
        {name: rng.randint(0, 1) for name in LKAS11_FIELDS} | {"CR_Lkas_StrToqReq": -37, "CF_Lkas_LdwsActivemode": 3,
                                                                "CF_Lkas_FcwOpt_USM": 5, "CF_Lkas_FusionState": 2},
    ]
    # (apply_steer, steer_req, cut_steer_temp, sys_warning, sys_state, l_depart, r_depart, count)
    commands = [
        (0, False, False, False, 1, 0, 0, 0),
        (384, True, False, False, 3, 0, 0, 5),
        (-384, True, False, True, 3, 1, 0, 15),
        (123, True, True, False, 4, 0, 2, 9),
        (-1, True, False, True, 2, 1, 1, 12),
    ]
    for seed in seeds:
        seed_bytes = pack(packer, "LKAS11", seed)
        for steer, req, cut, warning, state, left, right, count in commands:
            values = dict(seed)
            values.update({"CF_Lkas_LdwsSysState": state, "CF_Lkas_SysWarning": 3 if warning else 0,
                           "CF_Lkas_LdwsLHWarning": left, "CF_Lkas_LdwsRHWarning": right,
                           "CR_Lkas_StrToqReq": steer, "CF_Lkas_ActToi": int(req and not cut),
                           "CF_Lkas_ToiFlt": int(cut), "CF_Lkas_MsgCount": count % 16, "CF_Lkas_Chksum": 0})
            dat = pack(packer, "LKAS11", values)
            values["CF_Lkas_Chksum"] = (sum(dat[:6]) + dat[7]) % 256  # 상류 기본 분기
            yield seed_bytes, (steer, req, cut, warning, state, left, right, count), pack(packer, "LKAS11", values)


def clu11_cases(packer):
    seeds = [{"CF_Clu_Vanz": 0.0, "CF_Clu_CruiseSwMain": 0},
             {"CF_Clu_Vanz": 87.5, "CF_Clu_VanzDecimal": 0.25, "CF_Clu_CruiseSwMain": 1, "CF_Clu_RheostatLevel": 19,
              "CF_Clu_CluInfo": 1, "CF_Clu_ParityBit1": 1},
             {"CF_Clu_Vanz": 255.5, "CF_Clu_SPEED_UNIT": 1, "CF_Clu_DetentOut": 1, "CF_Clu_AmpInfo": 1,
              "CF_Clu_SldMainSW": 1}]
    for seed in seeds:
        seed_bytes = pack(packer, "CLU11", seed)
        for button, frame in ((0, 0), (1, 7), (2, 33), (4, 15)):
            values = dict(seed) | {"CF_Clu_CruiseSwState": button, "CF_Clu_AliveCnt1": frame % 0x10}
            yield seed_bytes, button, frame, pack(packer, "CLU11", values)


def mdps12_cases(packer):
    seeds = [{"CR_Mdps_StrColTq": 0, "CF_Mdps_ToiActive": 1},
             {"CR_Mdps_StrColTq": -213, "CF_Mdps_ToiActive": 1, "CF_Mdps_ToiFlt": 1, "CR_Mdps_StrTq": 3.5,
              "CR_Mdps_OutTq": -40.0, "CF_Mdps_MsgCount2": 77, "CF_Mdps_Chksum2": 200},
             {"CR_Mdps_StrColTq": 1000, "CF_Mdps_Def": 1, "CF_Mdps_FailStat": 1, "CF_Mdps_SErr": 1,
              "CR_Mdps_StrTq": -20.48, "CR_Mdps_OutTq": 204.7}]
    for seed in seeds:
        seed_bytes = pack(packer, "MDPS12", seed)
        for frame in (0, 1, 255, 256, 1000):
            values = dict(seed) | {"CF_Mdps_ToiActive": 0, "CF_Mdps_ToiUnavail": 1, "CF_Mdps_MsgCount2": frame % 0x100,
                                   "CF_Mdps_Chksum2": 0}
            dat = pack(packer, "MDPS12", values)
            values["CF_Mdps_Chksum2"] = sum(dat) % 256
            yield seed_bytes, frame, pack(packer, "MDPS12", values)


def torque_cases(lateral, rng: random.Random):
    limits = types.SimpleNamespace(**K7_LIMITS)
    fixed = [(0, 0, 0), (384, 0, 0), (384, 380, 0), (-384, -380, 0), (500, 384, 0), (384, 384, 150), (384, 384, -150),
             (-384, -384, 150), (100, -50, 0), (-100, 50, 0), (0, 200, 0), (0, -200, 0), (384, 300, -300)]
    randoms = [(rng.randint(-600, 600), rng.randint(-384, 384), rng.randint(-400, 400)) for _ in range(40)]
    for desired, last, driver in fixed + randoms:
        yield desired, last, driver, lateral.apply_driver_steer_torque_limits(desired, last, driver, limits)


def hexes(data: bytes) -> str:
    return "{" + ", ".join(f"0x{b:02x}" for b in data) + "}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("opendbc", type=Path, help="opendbc 저장소(opendbc_repo)")
    repo = parser.parse_args().opendbc.resolve()
    packer, lateral = load_opendbc(repo)
    commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=repo, capture_output=True,
                            text=True).stdout.strip() or "unknown"
    rng = random.Random(7)
    out = ["#pragma once", "",
           f"// tools/control/make_can_golden.py가 opendbc {commit}의 CANPacker와 상류 식으로 만든 골든 값이다. 손으로",
           "// 고치지 않고 다시 만든다. gtest_car.cc가 hyundai_can과 대조한다.", "",
           "#include <array>", "#include <cstdint>", "",
           "struct LkasGolden {", "    std::array<uint8_t, 8> seed;", "    int apply_steer;", "    bool steer_req;",
           "    bool cut_steer_temp;", "    bool sys_warning;", "    int sys_state;", "    int left_lane_depart;",
           "    int right_lane_depart;", "    int lkas_msg_count;",
           "    std::array<uint8_t, 8> expected;", "};", "inline constexpr LkasGolden kLkasGolden[] = {"]
    for seed, (steer, req, cut, warn, state, left, right, count), expected in lkas11_cases(packer, rng):
        flags = ", ".join(str(v).lower() for v in (req, cut, warn))
        out.append(f"    {{{hexes(seed)}, {steer}, {flags}, {state}, {left}, {right}, {count}, {hexes(expected)}}},")
    out += ["};", "", "struct Clu11Golden {", "    std::array<uint8_t, 4> seed;", "    int button;", "    int frame;",
            "    std::array<uint8_t, 4> expected;", "};", "inline constexpr Clu11Golden kClu11Golden[] = {"]
    for seed, button, frame, expected in clu11_cases(packer):
        out.append(f"    {{{hexes(seed)}, {button}, {frame}, {hexes(expected)}}},")
    out += ["};", "", "struct Mdps12Golden {", "    std::array<uint8_t, 8> seed;", "    int frame;",
            "    std::array<uint8_t, 8> expected;", "};", "inline constexpr Mdps12Golden kMdps12Golden[] = {"]
    for seed, frame, expected in mdps12_cases(packer):
        out.append(f"    {{{hexes(seed)}, {frame}, {hexes(expected)}}},")
    out += ["};", "", "// (원하는 토크, 지난 토크, 운전자 토크, 상류 apply_driver_steer_torque_limits)",
            "struct TorqueGolden {", "    int desired;", "    int last;", "    int driver;", "    int expected;", "};",
            "inline constexpr TorqueGolden kTorqueGolden[] = {"]
    for desired, last, driver, expected in torque_cases(lateral, rng):
        out.append(f"    {{{desired}, {last}, {driver}, {expected}}},")
    out += ["};"]
    print("\n".join(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
