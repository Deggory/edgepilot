# openpilot master supercombo → AX630C axmodel

MaixCAM2 런타임(`src/supercombo_model.cc`)이 쓰는 `models/supercombo.axmodel`을 만드는 과정.
공개 `driving_supercombo.onnx`에서 입력 큐(이미지·desire·특징 시프트 레지스터)를 떼어 낸
코어만 NPU에 올리고, 큐는 런타임이 CPU에서 관리한다(`src/model_temporal.h`).

1. 코어 추출(onnx 필요):
   `python3 extract_core.py driving_supercombo.onnx core_fp32.onnx`
   Pulsar2가 받지 못하는 연산(opset-20 Cast, GatherND, Where(-inf) 마스크, 2D LpNorm)을
   같은 값의 연산으로 바꾸고 fp16을 fp32로 올린다.
   이어서 출력 헤드를 나눈다(onnx 필요):
   `python3 split_outputs.py core_fp32.onnx core_split.onnx`
   원래 코어는 헤드 13개를 Concat 하나로 이어 내보내서, Pulsar2가 전체를 U16 눈금 하나(약 0.007)로
   양자화했다. plan yaw·yaw rate가 0 아니면 ±0.007로 나와 laneless 곡률이 계단마다 약 0.6 m/s²씩
   뛰었다. 헤드마다 출력으로 내보내고 plan은 위치·방향·표준편차로 나눈다. 런타임이
   `src/model_output_assembly.h`로 다시 모은다. 변환 설정의 `input`은 `core_split.onnx`다.
2. 보정·평가 데이터(onnxruntime 필요):
   `python3 make_core_data.py` → `calib/*.tar`, `eval/`. PTQ 샘플은 `models/ptq`,
   평가 묶음은 `QEXP094_DIR`(K230 0.9.4 평가, 저장소 밖)에서 읽는다.
   MaixCAM2 녹화로 만들려면 `python3 make_m2_data.py calib m2_calib_20261004.json calib_m2`.
   보드가 코어에 넣는 입력을 그대로 만든다: 녹화 H.264 → 장치 워프(MaixCAM2 내부 파라미터,
   녹화 보정값) → 탑마다 t−4·t 이미지, controlsd가 보낸 desire 펄스, fp32 코어를 차례로 돌린
   진짜 특징 이력. `make_core_data.py`는 K230 카메라 표본에 t−1을 t−4 대신 쓰고 특징을 아무
   프레임에서 가져와, 그 U16 모델이 2026-10-04 녹화의 일부 녹색 신호에서 코어의 반응(경로 열림,
   가속 확률)을 잃었다. 같은 도구의 `eval`은 보드 비교용 입력과 fp32 출력을 만든다
   (보드에서 `run_axmodel_assembled.py`). 영상 세그먼트는 `video_dir`(보드 녹화에서 받은 사본),
   이벤트는 `routes_dir`에서 읽는다.
3. 변환(Pulsar2 6.0-lite, docker amd64):
   작업 디렉터리를 `/data`로 마운트하고 `pulsar2 build --config /data/pulsar2_u16_u8in.json`.
   NPU 코어 하나용(NPU1: 다른 코어는 AI-ISP가 쓴다), U16 전체(Conv 가중치는 S8: AX620E는
   FP32 가중치 설정을 무시한다), SmoothQuant(`enable_smooth_quant`), 이미지 입력은
   uint8(`input_processors`)이다. 2026-10-04 비교(보드, 같은 입력의 fp32 대비): SmoothQuant가
   MaixCAM2 주행 구간 laneless 횡가속 오차 평균 0.020 → 0.018, p95 0.063 → 0.057 m/s², 특징
   cosine 0.947 → 0.952, 녹색 신호 8곳 중 가속 확률 반응 4 → 6. MaixCAM2 녹화 보정(`make_m2_data.py`)은
   MinMax 범위를 넓혀 오히려 나빴고, highest_mix_precision은 빌드가 안 되며, EasyQuant는 Docker
   메모리 9.7 GB로 모자랐다. 결과 `build/core.axmodel`을 저장소의
   `models/supercombo.axmodel`로 넣고 `models/manifest.sha256`을 갱신하면
   `scripts/upload_to_board.sh`가 보드로 보낸다.

2026-09-29 분할 빌드(보드, 평가 552프레임, fp32 대비): laneless 목표 횡가속 오차 평균 0.109 → 0.054 m/s²
(p95 0.35 → 0.22), yaw 서로 다른 값 12 → 568개, 차선·선행차·plan 위치는 그대로다.

보정 데이터의 desire에는 펄스가 들어 있어야 한다(`make_core_data.py`). 예전 모델은 전부 0으로
보정해 desire 입력 범위가 [0, 0]이었고, NPU 모델이 차선 변경 명령에 전혀 반응하지 않았다
(2026-09-27: 보드에서 펄스를 넣어도 laneChangeLeft 확률 0.00, 새 모델은 0.99이고 desire가 없을 때
출력은 같다, plan y 차이 0.0006 m).

참고: Elu는 Pulsar2 6.0/7.0 모두 U16에서 잘못 컴파일돼서(0.9.4 모델) master 모델로 옮겼다.
보드 NPU 추론은 코어 하나에서 약 15.5 ms다.
