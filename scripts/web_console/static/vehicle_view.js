// 차량 특성 탭: 차의 성질인 조향 값들(steering.json의 tab="vehicle" 항목). 학습기(paramsd·torqued·lagd)마다 한 묶음이고,
// 묶음 머리에서 제어에 학습값을 쓸지(자동 학습) 수동값을 쓸지(수동) 고른다. 값마다 카드 하나에 지금 제어에 쓰는 값과
// 그 출처, 학습값, 수동값(바로 고친다)과 자동 학습 중 그 쓰임을 함께 둔다. 그 아래 카메라 보정, 최근 10분 추이, 학습
// 입력, 고정 제원. 1초마다 /api/learners를 읽는다.
import {locale, t} from "./i18n.js";
import {clampToMeta, defaultOf, metaFor, params, saveParam, sectionName, valueOf} from "./param_state.js";
import {effectsDetails, formatValue, paramCard, paramEditor} from "./param_editor.js";
import {
  actionButton, api, badges, clock, confirmAction, deg, el, infoTable, liveCard, ms, note, num, poller, section, sgn, svg,
  toast,
} from "./ui.js";

const GRAVITY = 9.81;
const steering = () => params().params.steering;
const restored = flag => (flag ? t("vehicle.restored") : t("vehicle.fresh"));
const lagConfirmed = localization => Boolean(localization) && localization.state.lag_status === 1;
// lagd 유효 블록 수: 5개부터 확정이고 50개까지 는다
const lagBlocks = blocks => t(blocks >= 5 ? "vehicle.blocks_confirmed" : "vehicle.blocks", {blocks});
const torqueCalculable = s => s.bucket_points.every(n => n > 0) && s.lat_accel_factor_raw !== 0;

// torqued 값의 원시값(필터 전)과 허용 폭. 원시값이 폭 밖이면 주의 톤.
function torqueDetail(s, raw, prior, width, digits) {
  const calculable = torqueCalculable(s);
  const outside = calculable && (raw < prior * (1 - width) || raw > prior * (1 + width));
  return [t("vehicle.torque_detail", {raw: calculable ? num(raw, digits) : "–", low: num(prior * (1 - width), digits),
                                       high: num(prior * (1 + width), digits)}),
          outside ? "warn" : ""];
}

// 값마다 학습값을 읽는 법. live = {learner: LearnerState, localization: locationd 상태 또는 null}.
//   learned(live)            학습값(파라미터 단위), 없으면 NaN. valid(live)는 그 값이 유효한가
//   detail(live)             학습값 옆 작은 글씨(편차·범위), 또는 [글씨, 톤]
//   digits, signed           학습값과 지금 쓰는 값의 자릿수·부호. 학습값을 수동값으로 저장할 때도 이 자릿수
//   using(live, manual)      자동 학습 중 실제로 쓰는 값(없으면 학습값). both면 수동값도 함께 쓰인다
//   learnedText, manualText  값 대신 보여 줄 글씨
//   adopt                    수동값으로 저장할 값(없으면 학습값), false면 버튼 없음. adoptLabel()은 그 값의 이름
//   title()                  카드 제목(없으면 항목 이름)
const LIVE = {
  steer_ratio: {
    digits: 2,
    learned: ({learner: s}) => s.steer_ratio,
    valid: ({learner: s}) => s.flags.steer_ratio_valid,
    detail: ({learner: s}) => t("vehicle.ratio_detail", {std: num(s.steer_ratio_std, 2), low: num(s.prior_steer_ratio * 0.5, 1),
                                                         high: num(s.prior_steer_ratio * 2, 1)}),
  },
  tire_stiffness_factor: {
    digits: 2,
    learned: ({learner: s}) => s.stiffness_factor,
    learnedText: value => `×${num(value, 3)}`,
    valid: ({learner: s}) => s.flags.stiffness_valid,
    detail: ({learner: s}) => t("vehicle.stiffness_detail", {std: num(s.stiffness_factor_std, 3)}),
    using: ({learner: s}, manual) => manual * s.stiffness_factor,
    both: true,
    adopt: false,
  },
  angle_offset_deg: {
    digits: 2,
    signed: true,
    learned: ({learner: s}) => s.angle_offset_deg,
    valid: ({learner: s}) => s.flags.offset_valid,
    detail: ({learner: s}) => t("vehicle.offset_detail", {average: sgn(s.angle_offset_average_deg, 2),
                                                          std: num(deg(s.angle_offset_average_std), 2),
                                                          fast: num(deg(s.angle_offset_fast_std), 2)}),
    adopt: ({learner: s}) => s.angle_offset_average_deg,
    adoptLabel: () => t("vehicle.adopt_average"),
  },
  live_bank_compensation: {
    title: () => t("vehicle.bank_title"),
    digits: 2,
    signed: true,
    learned: ({learner: s}) => deg(s.roll_rad),
    learnedText: value => t("vehicle.roll_value", {roll: sgn(value, 2)}),
    valid: ({learner: s}) => s.flags.vehicle_valid,
    detail: () => t("vehicle.roll_detail"),
    manualText: ({learner: s}, on) => (on ? t("vehicle.bank_value", {bank: sgn(deg(-s.road_bank_lat_accel / GRAVITY), 2)})
      : t("vehicle.bank_off")),
    adopt: false,
  },
  torque_lat_accel_factor: {
    digits: 2,
    learned: ({learner: s}) => s.lat_accel_factor,
    valid: ({learner: s}) => s.flags.torque_valid,
    detail: ({learner: s}) => torqueDetail(s, s.lat_accel_factor_raw, s.prior_lat_accel_factor, 0.3, 2),
  },
  torque_friction: {
    digits: 3,
    learned: ({learner: s}) => s.friction,
    valid: ({learner: s}) => s.flags.torque_valid,
    detail: ({learner: s}) => torqueDetail(s, s.friction_raw, s.prior_friction, 0.5, 3),
  },
  torque_lat_accel_offset: {
    digits: 3,
    signed: true,
    learned: ({learner: s}) => s.lat_accel_offset,
    valid: ({learner: s}) => s.flags.torque_valid,
    detail: ({learner: s}) => t("vehicle.raw_detail", {raw: torqueCalculable(s) ? sgn(s.lat_accel_offset_raw, 3) : "–"}),
  },
  steer_actuator_delay: {
    digits: 3,
    learned: ({localization}) => (lagConfirmed(localization) ? localization.state.lateral_delay_s : NaN),
    learnedText: (value, {localization}) => (Number.isFinite(value) ? num(value, 3) : localization ? t("vehicle.lag_pending") : "–"),
    valid: ({localization}) => lagConfirmed(localization),
    detail: ({localization}) => (!localization ? t("vehicle.no_locationd_state")
      : t("vehicle.lag_detail", {estimate: num(localization.state.lag_estimate_s, 3), std: ms(localization.state.lag_estimate_std_s),
                                 blocks: lagBlocks(localization.state.lag_valid_blocks)})),
    using: ({learner: s}, manual) => (s.plan_delay_s > 0 ? s.plan_delay_s : manual),  // 실제로 쓴 경로 지연
  },
};

const LAG_STATUS = {0: ["vehicle.lag_pending", "warn"], 1: ["vehicle.lag_estimated", "good"], 2: ["vehicle.invalid", "bad"]};
// 학습기마다: 지금 제어에 학습값을 쓰고 있는가(controlsd가 LearnerState 플래그로 알린다), 쓰기 전의 상태 이름, 상태
// 배지, 한 줄 요약.
const LEARNERS = {
  paramsd: {
    inUse: ({learner: s}) => s.flags.use_vehicle,
    waiting: () => t("vehicle.waiting"),
    status: ({learner: s}) => [
      s.flags.vehicle_valid ? [t("vehicle.valid"), "good"] : [t("vehicle.invalid"), "bad"],
      s.flags.vehicle_inputs_ok ? null : [t("vehicle.inputs_lost"), "warn"],
      s.flags.sensor_valid ? null : [t("vehicle.sensor_mismatch"), "warn"],
      [t("vehicle.input_source", {source: s.flags.localizer_inputs ? "locationd" : "ESP12"}), "muted"],
    ],
    summary: ({learner: s}) => t("vehicle.paramsd_summary", {bias: sgn(deg(s.yaw_bias_rad_s), 3),
                                                             restored: restored(s.flags.vehicle_restored)}),
  },
  torqued: {
    inUse: ({learner: s}) => s.flags.use_torque,
    waiting: () => t("vehicle.waiting"),
    status: ({learner: s}) => [
      s.flags.torque_valid ? [t("vehicle.valid"), "good"] : [t("vehicle.learning", {percent: s.cal_perc}), "warn"],
      s.flags.torque_inputs_ok ? null : [t("vehicle.inputs_lost"), "warn"],
    ],
    summary: ({learner: s}) => t("vehicle.torqued_summary", {
      points: s.total_bucket_points.toLocaleString(locale), decay: num(s.decay, 1),
      count: Math.max(0, Math.round(s.max_resets) - 1),
      restored: s.flags.torque_partly_restored ? t("vehicle.partly_restored") : restored(s.flags.torque_restored),
    }),
  },
  lagd: {
    inUse: ({learner: s}) => s.flags.use_delay,
    waiting: () => t("vehicle.lag_waiting"),
    status: ({localization}) => {
      if (!localization) return [[t("vehicle.no_locationd"), "bad"]];
      const s = localization.state, f = s.flags, inputs = s.input_flags;
      const lag = LAG_STATUS[s.lag_status];
      return [
        lag ? [t(lag[0]), lag[1]] : [String(s.lag_status), "muted"],
        [lagBlocks(s.lag_valid_blocks), s.lag_valid_blocks >= 5 ? "accent" : "muted"],
        f.filter_valid && f.inputs_ok && f.sensors_ok && f.posenet_ok ? [t("vehicle.pose_ok"), "good"] : [t("vehicle.pose_invalid"), "bad"],
        inputs.accel_invalid ? [t("vehicle.accel_rejected"), "warn"] : null,
        inputs.gyro_invalid ? [t("vehicle.gyro_rejected"), "warn"] : null,
        inputs.camera_invalid ? [t("vehicle.camera_rejected"), "warn"] : null,
        inputs.camera_guarded ? [t("vehicle.camera_guarded"), "muted"] : null,
        localization.age_s > 2 ? [t("vehicle.ago", {seconds: Math.round(localization.age_s)}), "bad"] : null,
      ];
    },
    summary: ({localization}) => {
      if (!localization) return t("vehicle.locationd_silent");
      const s = localization.state;
      return t("vehicle.lagd_summary", {
        yaw_rate: sgn(deg(s.angular_velocity_calib[2]), 2), roll: sgn(deg(s.orientation_calib[0]), 2),
        pitch: sgn(deg(s.orientation_calib[1]), 2), points: s.lag_points, restored: restored(s.flags.lag_restored),
      });
    },
  },
};

// ------------------------------------------------------------ 학습기 묶음
// 묶음 머리의 출처 선택: 자동 학습(학습 스위치 켬) | 수동(끔). 고르면 곧바로 저장하고 onChange를 부른다.
function sourceControl(switchKey, label, onChange) {
  const auto = el("button", {type: "button", text: t("vehicle.auto")});
  const manual = el("button", {type: "button", text: t("vehicle.manual")});
  const element = el("div.segmented", {role: "group", "aria-label": t("vehicle.source_of", {label})}, auto, manual);
  const sync = () => {
    const on = steering()[switchKey] === true;
    auto.setAttribute("aria-pressed", String(on));
    manual.setAttribute("aria-pressed", String(!on));
  };
  const choose = async on => {
    if ((steering()[switchKey] === true) === on) return;
    auto.disabled = manual.disabled = true;
    try {
      await saveParam("steering", switchKey, on);
      toast(`${label}: ${on ? t("vehicle.auto") : t("vehicle.manual")} · ${clock()}`);
    } catch (error) {
      toast(`${label}: ${error.message}`, {failed: true});
    } finally {
      auto.disabled = manual.disabled = false;
      sync();
      onChange();
    }
  };
  auto.addEventListener("click", () => choose(true));
  manual.addEventListener("click", () => choose(false));
  sync();
  return {element, sync};
}

// 값 카드: 지금 쓰는 값과 출처, 학습값, 수동값(편집기)과 자동 학습 중 그 쓰임, 학습값을 수동값으로 저장, 설명.
function quantityCard(key, switchKey, learner) {
  const meta = metaFor("steering", key);
  const spec = LIVE[key];
  const card = el("article.card.quantity", {dataset: {key}});
  let live = null;
  const editor = paramEditor("steering", key, {card, onSaved: () => render()});
  const format = (value, digits = spec.digits) => (spec.signed ? sgn : num)(value, digits);
  const source = el("span.badge");
  const now = el("span.quantity-now");
  const learnedValue = el("span.quantity-value");
  const learnedDetail = el("span.quantity-detail");
  const learnedRow = el("div.quantity-row", {}, el("span.quantity-label", {text: t("vehicle.learned")}), learnedValue, learnedDetail);
  const fallback = defaultOf("steering", key);
  const manualRow = el("div.quantity-row.manual",
    {},
    el("span.quantity-label", {text: t("vehicle.manual_value")}),
    el("span.quantity-foot",
      {},
      fallback === undefined ? null : el("span", {text: t("params.default", {value: formatValue(fallback, meta)})}),
      editor.modifiedBadge, editor.resetButton, editor.status),
    editor.control.element);
  const role = el("p.card-note.prior");
  const adopt = spec.adopt === false ? null : el("button.adopt-button", {type: "button", onclick: () => adoptLearned()});
  const adoptValue = () => (typeof spec.adopt === "function" ? spec.adopt(live) : spec.learned(live));
  const showNow = (text, unit = "") => now.replaceChildren(text, unit ? el("small", {text: ` ${unit}`}) : "");
  // 수동값을 쓰는 중일 때의 큰 글씨: manualText가 있으면 그것, 켜고 끄는 값이면 켜짐·꺼짐, 숫자면 단위와 함께
  const showManual = manual => {
    if (spec.manualText && live) showNow(spec.manualText(live, manual));
    else if (typeof manual === "boolean") showNow(manual ? t("common.on") : t("common.off"));
    else showNow(String(manual), meta.unit);
  };

  const updateAdopt = valid => {
    if (!adopt) return;
    const current = valueOf("steering", key);
    const target = live ? clampToMeta(adoptValue(), meta, spec.digits) : NaN;
    const same = Number(current) === Number(target);
    adopt.replaceChildren(
      el("span", {text: spec.adoptLabel ? t("vehicle.adopt_named", {name: spec.adoptLabel()}) : t("vehicle.adopt")}),
      el("small", {text: !live || !valid || !Number.isFinite(target) ? t("vehicle.adopt_when_valid")
        : same ? t("vehicle.adopt_same") : `${current} → ${target}`}));
    adopt.disabled = !live || !valid || same || !Number.isFinite(target);
  };

  const adoptLearned = async () => {
    if (!live) return;
    const current = valueOf("steering", key);
    const target = clampToMeta(adoptValue(), meta, spec.digits);
    const auto = steering()[switchKey] === true;
    if (!confirmAction([`${meta.label} (${key})`, `${current} → ${target}`, "",
                        auto ? t("vehicle.role", {role: meta.learned_role}) : t("vehicle.adopt_manual_now"),
                        meta.caution || null])) return;
    await editor.save(target);
  };

  function render() {
    const auto = steering()[switchKey] === true;
    const manual = valueOf("steering", key);
    role.textContent = auto && meta.learned_role ? t("vehicle.role", {role: meta.learned_role}) : "";
    role.hidden = !role.textContent;
    if (!live) {
      source.className = "badge muted";
      source.textContent = auto ? t("vehicle.no_state") : t("vehicle.manual_value");
      if (auto) showNow("–");
      else showManual(manual);
      learnedValue.textContent = "–";
      learnedValue.className = "quantity-value";
      learnedDetail.textContent = t("vehicle.no_controlsd_state");
      learnedRow.classList.remove("in-use");
      manualRow.classList.toggle("in-use", !auto);
      updateAdopt(false);
      return;
    }
    const inUse = auto && learner.inUse(live);
    const learned = spec.learned(live);
    const valid = spec.valid(live);
    learnedValue.textContent = spec.learnedText ? spec.learnedText(learned, live) : format(learned);
    learnedValue.className = `quantity-value${valid ? "" : " bad"}`;
    const detail = spec.detail(live);
    const [detailText, detailTone] = Array.isArray(detail) ? detail : [detail, ""];
    learnedDetail.textContent = detailText;
    learnedDetail.className = `quantity-detail${detailTone ? ` ${detailTone}` : ""}`;
    if (inUse) {
      const value = spec.using ? spec.using(live, manual) : learned;
      if (spec.learnedText && !spec.using) showNow(spec.learnedText(value, live));
      else showNow(format(value), meta.unit);
      source.className = "badge good";
      source.textContent = spec.both ? t("vehicle.manual_times_learned") : t("vehicle.learned");
    } else {
      showManual(manual);
      source.className = `badge ${auto ? "warn" : "muted"}`;
      source.textContent = auto ? learner.waiting() : t("vehicle.manual_value");
    }
    learnedRow.classList.toggle("in-use", inUse);
    manualRow.classList.toggle("in-use", !inUse || Boolean(spec.both));
    updateAdopt(valid);
  }

  card.append(...[
    el("header.card-head", {}, el("h3.card-title", {text: spec.title ? spec.title() : meta.label}), source),
    el("div.quantity-now-row", {}, now, el("span.quantity-caption", {text: t("vehicle.now")})),
    learnedRow,
    manualRow,
    role,
    meta.caution ? note(meta.caution, "warn") : null,
    adopt,
    effectsDetails(meta, typeof valueOf("steering", key) === "boolean",
                   {summary: t("vehicle.description"), lead: el("p.param-description", {text: meta.description})}),
  ].filter(Boolean));
  render();
  return {
    element: card,
    update(next) {
      live = next;
      render();
    },
    refresh() {
      editor.refresh();
      render();
    },
  };
}

// 학습기 묶음 하나: 이름·학습기·설명, 출처 선택, 상태 배지, 값 카드들, 한 줄 요약.
function learnerGroup(id, spec, keys) {
  const name = sectionName(id);
  const learner = LEARNERS[spec.learner];
  const cards = keys.map(key => quantityCard(key, spec.switch, learner));
  const statusRow = el("div.badge-row");
  const summary = el("p.group-summary");
  let live = null;
  const update = next => {
    live = next;
    const auto = steering()[spec.switch] === true;
    statusRow.replaceChildren(...badges([
      !live ? [t("vehicle.no_state"), "muted"]
        : !auto ? [t("vehicle.using_manual"), "muted"]
        : learner.inUse(live) ? [t("vehicle.using_learned"), "accent"] : [learner.waiting(), "warn"],
      ...(live ? learner.status(live) : []),
    ]));
    summary.textContent = live ? learner.summary(live) : t("vehicle.no_learner_state");
    cards.forEach(card => card.update(live));
  };
  const source = sourceControl(spec.switch, name, () => update(live));
  return {
    element: el("section.section.learner-group",
      {},
      el("div.group-head",
        {},
        el("div.group-title",
          {},
          el("h2.section-title", {}, name, el("span", {text: spec.learner})),
          el("p.group-description", {text: metaFor("steering", spec.switch).description})),
        source.element),
      el("div.group-status", {}, statusRow),
      el("div.card-grid", {}, cards.map(card => card.element)),
      summary),
    update,
    refresh() {
      source.sync();
      cards.forEach(card => card.refresh());
      update(live);
    },
  };
}

// ------------------------------------------------------------ 카메라 보정
// 보정 상태마다 [배지 문구 키, 톤]. 포즈 보정기의 상태는 표에 calibration.pose.<상태>로 쓴다.
const CALIBRATION_STATUS = {
  calibrated: ["calibration.status.calibrated", "good"], uncalibrated: ["calibration.status.uncalibrated", "warn"],
  invalid: ["calibration.status.invalid", "bad"], recalibrating: ["calibration.status.recalibrating", "warn"],
};
const statusBadge = status => (CALIBRATION_STATUS[status] ? [t(CALIBRATION_STATUS[status][0]), CALIBRATION_STATUS[status][1]]
  : [status, "muted"]);
const poseStatus = status => (status in CALIBRATION_STATUS ? t(`calibration.pose.${status}`) : status);
const sourceText = source => (source === "flow" ? t("calibration.flow") : t("calibration.pose_source"));

// 워프 보정(modeld calibration)과 그 출처. 출처 버튼은 display.json의 calibration_use_flow를 바꾼다(결합 중에는 서버가
// 막고 modeld도 해제를 기다린다). 표는 워프에 쓴 값, 광류 추정, 모델 포즈 보정기 자신의 추정과 둘의 차이(모델 pose의
// 치우침)다. 초기화는 두 보정을 다 지운다.
function calibrationCard() {
  let latest = null;
  let saving = false;
  const button = actionButton({icon: "↺", danger: true, onclick: () => reset()});
  const flowButton = el("button", {type: "button", text: t("calibration.use_flow")});
  const poseButton = el("button", {type: "button", text: t("calibration.use_pose")});
  const sourceControl = el("div.segmented", {role: "group", "aria-label": t("calibration.source")}, flowButton, poseButton);
  const card = liveCard(t("calibration.card"), {control: sourceControl, actions: [button]});

  const choose = async useFlow => {
    const c = latest;
    if (!c || !c.available || saving || Boolean(c.use_flow) === useFlow) return;
    saving = true;
    update(c);
    try {
      latest = await api.post("/api/calibration/source", {use_flow: useFlow});
      toast(`${t("calibration.source")}: ${useFlow ? t("calibration.use_flow") : t("calibration.use_pose")} · ${clock()}`);
    } catch (error) {
      toast(`${t("calibration.source")}: ${error.message}`, {failed: true});
    } finally {
      saving = false;
      update(latest);
    }
  };
  flowButton.addEventListener("click", () => choose(true));
  poseButton.addEventListener("click", () => choose(false));

  const reset = async () => {
    const c = latest;
    if (!confirmAction([t("calibration.confirm"),
                        c && c.available
                          ? t("calibration.confirm_now", {pitch: sgn(c.rpy_deg[1], 2), yaw: sgn(c.rpy_deg[2], 2), source: sourceText(c.source)})
                          : t("calibration.confirm_no_state"),
                        "", t("calibration.confirm_effect"), t("calibration.confirm_after")])) return;
    button.disabled = true;
    try {
      update(await api.post("/api/calibration/reset"));
      toast(t("calibration.reset_requested"));
    } catch (error) {
      toast(t("calibration.reset_failed", {error: error.message}), {failed: true});
      button.disabled = false;
    }
  };

  const update = c => {
    latest = c;
    const blocked = Boolean(c && c.engaged);
    flowButton.setAttribute("aria-pressed", String(Boolean(c && c.available && c.use_flow)));
    poseButton.setAttribute("aria-pressed", String(Boolean(c && c.available && !c.use_flow)));
    flowButton.disabled = poseButton.disabled = saving || blocked || !(c && c.available);
    sourceControl.title = blocked ? t("calibration.blocked_source") : "";
    if (!c || !c.available) {
      card.update([[t("vehicle.no_state"), "muted"]], [note(t("calibration.no_state"))]);
    } else {
      const [r, p, y] = c.rpy_deg;
      const flow = c.flow || {status: "off", sd_deg: [0, 0], window: {}};
      const pose = c.pose || {status: c.status, valid_blocks: c.valid_blocks, rpy_deg: c.rpy_deg};
      const hasFlow = flow.estimate_blocks > 0;
      const estimate = flow.status === "estimate";
      const dp = pose.rpy_deg[1] - flow.pitch_deg, dy = pose.rpy_deg[2] - flow.yaw_deg;
      const w = flow.window;
      card.update([statusBadge(c.status),
                   c.source === "flow" ? [t("calibration.flow_applied"), "accent"] : [t("calibration.pose_applied"), "muted"],
                   c.switch_pending ? [t("calibration.switch_pending"), "warn"] : null,
                   c.ramping ? [t("calibration.ramping"), "accent"] : null,
                   flow.status === "collecting" ? [t("calibration.flow_collecting", {blocks: flow.estimate_blocks}), "warn"]
                     : estimate ? [t("calibration.flow_blocks", {blocks: flow.estimate_blocks}), "muted"] : null,
                   c.age_s > 2 ? [t("vehicle.ago", {seconds: Math.round(c.age_s)}), "bad"] : null], [
        infoTable(["", "pitch (°)", "yaw (°)", ""], [
          [t("calibration.row_warp"), sgn(p, 2), sgn(y, 2), t("calibration.row_warp_note", {roll: sgn(r, 2), source: sourceText(c.source)})],
          [t("calibration.row_flow"), hasFlow ? sgn(flow.pitch_deg, 2) : "—", hasFlow ? sgn(flow.yaw_deg, 2) : "—",
           hasFlow ? t("calibration.row_flow_note", {blocks: flow.estimate_blocks, pitch_sd: num(flow.sd_deg[0], 2), yaw_sd: num(flow.sd_deg[1], 2)})
                   : flow.status === "off" ? t("calibration.flow_off") : t("calibration.collecting")],
          [t("calibration.row_pose"), sgn(pose.rpy_deg[1], 2), sgn(pose.rpy_deg[2], 2),
           t("calibration.row_pose_note", {blocks: pose.valid_blocks,
                                           status: c.source === "flow" || c.ramping ? t("calibration.paused") : poseStatus(pose.status)})],
          [t("calibration.row_bias"), estimate ? sgn(dp, 2) : "—", estimate ? [sgn(dy, 2), Math.abs(dy) > 0.2 ? "warn" : ""] : "—",
           t("calibration.row_bias_note")],
        ]),
        c.use_flow && !c.flow_usable ? note(t("calibration.flow_not_usable"), "warn") : null,
        flow.windows > 0
          ? note(t("calibration.windows", {windows: flow.windows, rejected: flow.rejected_windows, tracks: w.tracks, inliers: w.inliers,
                                           rotation: num(w.rotation_deg, 3), parallax: num(w.parallax_px, 1),
                                           changes: flow.mount_changes})) : null,
        note(t("calibration.how")),
      ]);
    }
    button.setContent(c && c.reset_pending ? t("calibration.reset_pending") : t("calibration.reset"),
                      blocked ? t("calibration.blocked_reset") : t("calibration.reset_note"));
    button.disabled = blocked;
  };
  return {element: card.element, update};
}

// ------------------------------------------------------------ 추이
// title은 문구 키
const TREND_CHARTS = [
  {title: "vehicle.trend_ratio", series: [[1, "main"]], ref: m => [m.steer_ratio], span: 0.2, digits: 2},
  {title: "vehicle.trend_offset", series: [[2, "main"], [3, "thin"]], ref: m => [m.angle_offset_deg], span: 0.2, digits: 2},
  {title: "vehicle.trend_roll", series: [[4, "main"], [5, "dash"]], ref: () => [], span: 0.5, digits: 2},
  {title: "vehicle.trend_torque", series: [[7, "main"], [6, "raw"]],
   ref: (m, s) => [s.prior_lat_accel_factor, s.prior_lat_accel_factor * 0.7, s.prior_lat_accel_factor * 1.3], span: 0.2, digits: 2},
];

function trendChart(chart, rows, windowS, manual, state) {
  const end = rows.length ? rows[rows.length - 1][0] : 0;
  const shown = rows.filter(row => row[0] >= end - windowS);
  const valid = (row, index, style) => Number.isFinite(row[index]) && !(style === "raw" && row[index] === 0);
  const refs = chart.ref(manual, state).filter(Number.isFinite);
  const values = [...refs];
  for (const [index, style] of chart.series) for (const row of shown) if (valid(row, index, style)) values.push(row[index]);
  let lo = values.length ? Math.min(...values) : 0, hi = values.length ? Math.max(...values) : 1;
  if (hi - lo < chart.span) {
    const mid = (hi + lo) / 2;
    lo = mid - chart.span / 2;
    hi = mid + chart.span / 2;
  }
  const pad = (hi - lo) * 0.08;
  lo -= pad;
  hi += pad;
  const W = 300, H = 90;
  const x = time => ((time - (end - windowS)) / windowS) * W;
  const y = v => H - ((v - lo) / (hi - lo)) * H;
  const title = t(chart.title);
  const plot = svg("svg", {viewBox: `0 0 ${W} ${H}`, preserveAspectRatio: "none", role: "img", "aria-label": title});
  refs.forEach((value, i) => plot.append(svg("line", {
    x1: 0, x2: W, y1: y(value), y2: y(value), stroke: i === 0 ? "#8a949b" : "#5b646b", "stroke-dasharray": i === 0 ? "5 4" : "2 4",
    "vector-effect": "non-scaling-stroke",
  })));
  const stroke = {main: "#58a6e7", thin: "#a5adb4", dash: "#a5adb4", raw: "#efb85b"};
  for (const [index, style] of chart.series) {
    const points = shown.filter(row => valid(row, index, style)).map(row => `${x(row[0]).toFixed(1)},${y(row[index]).toFixed(1)}`);
    if (points.length < 2) continue;
    plot.append(svg("polyline", {
      points: points.join(" "), fill: "none", stroke: stroke[style], "stroke-width": style === "main" ? 2 : 1.2,
      "vector-effect": "non-scaling-stroke", ...(style === "dash" ? {"stroke-dasharray": "4 3"} : {}),
    }));
  }
  const last = shown.length ? shown[shown.length - 1] : null;
  return el("article.card.trend",
    {},
    el("div.trend-head", {}, el("span", {text: title}),
       el("span.trend-now", {text: last ? chart.series.map(([index, style]) => (valid(last, index, style) ? num(last[index], chart.digits) : "–")).join(" / ") : "–"})),
    plot,
    el("div.trend-range", {}, el("span", {text: `${num(lo, chart.digits)} ~ ${num(hi, chart.digits)}`}),
       el("span", {text: refs.length ? t("vehicle.trend_dashed", {value: num(refs[0], chart.digits)}) : t("vehicle.trend_window")})));
}

// ------------------------------------------------------------ 탭
export function createVehicleView() {
  const tab = params().vehicle_tab;
  const keysOf = name => Object.keys(steering()).filter(key => {
    const meta = metaFor("steering", key);
    return meta.tab === "vehicle" && meta.section === name;
  });
  const groups = tab.sections.filter(id => tab.learners[id]).map(id => {
    const spec = tab.learners[id];
    return learnerGroup(id, spec, keysOf(id).filter(key => key !== spec.switch));
  });
  const plain = tab.sections.filter(id => !tab.learners[id]).map(id => ({id, cards: keysOf(id).map(key => paramCard("steering", key))}));
  const calibration = calibrationCard();
  const notice = el("div");
  const trends = el("div.card-grid");
  let rows = [];
  let windowS = 600;
  let trendLoaded = false;

  const update = data => {
    calibration.update(data.calibration);
    if (!data.available) {
      notice.replaceChildren(el("p.group-note.warn", {text: t("vehicle.no_learners")}));
      groups.forEach(group => group.update(null));
      return;
    }
    notice.replaceChildren(data.age_s > 2
      ? el("p.group-note.warn", {text: t("vehicle.learners_stale", {seconds: Math.round(data.age_s)})}) : "");
    const live = {learner: data.state, localization: data.localization && data.localization.available ? data.localization : null};
    groups.forEach(group => group.update(live));
    const row = data.trend_row;
    if (row && rows.length && row[0] < rows[rows.length - 1][0]) rows = [];
    if (row && (!rows.length || row[0] > rows[rows.length - 1][0])) {
      rows.push(row);
      while (rows.length && rows[0][0] < row[0] - windowS) rows.shift();
    }
    trends.replaceChildren(...TREND_CHARTS.map(chart => trendChart(chart, rows, windowS, steering(), data.state)));
  };

  const polling = poller(async isCurrent => {
    try {
      if (!trendLoaded) {
        const trend = await api.get("/api/learners/trend");
        rows = trend.rows;
        windowS = trend.window_s;
        trendLoaded = true;
      }
      const data = await api.get("/api/learners");
      if (isCurrent()) update(data);
    } catch (error) {
      if (isCurrent()) notice.replaceChildren(el("p.group-note.warn", {text: t("vehicle.learners_failed", {error: error.message})}));
    }
  }, 1000);

  return {
    element: el("div.view",
      {},
      el("p.group-note", {text: tab.note}),
      notice,
      groups.map(group => group.element),
      section(t("calibration.section"), [calibration.element]),
      el("section.section", {}, el("h2.section-title", {text: t("vehicle.trends")}), trends),
      plain.map(({id, cards}) => section(sectionName(id), cards, {count: cards.length})),
      el("p.file-path", {text: t("params.file", {path: params().paths.steering})})),
    start() {
      // 가려져 있던 동안의 추이는 서버에서 다시 받는다
      trendLoaded = false;
      groups.forEach(group => group.refresh());
      plain.forEach(({cards}) => cards.forEach(card => card.refresh()));
      polling.start();
    },
    stop: polling.stop,
  };
}
