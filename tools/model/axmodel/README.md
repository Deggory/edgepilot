# openpilot master supercombo → AX630C axmodel

MaixCAM2 런타임(`src/supercombo_model.cc`)이 쓰는 `models/supercombo.axmodel`을 만드는 과정.
공개 `driving_supercombo.onnx`에서 입력 큐(이미지·desire·특징 시프트 레지스터)를 떼어 낸
코어만 NPU에 올리고, 큐는 런타임이 CPU에서 관리한다(`src/model_temporal.h`).

1. 코어 추출(onnx 필요):
   `python3 extract_core.py driving_supercombo.onnx core_fp32.onnx`
   Pulsar2가 받지 못하는 연산(opset-20 Cast, GatherND, Where(-inf) 마스크, 2D LpNorm)을
   같은 값의 연산으로 바꾸고 fp16을 fp32로 올린다.
2. 보정·평가 데이터(onnxruntime 필요):
   `python3 make_core_data.py` → `calib/*.tar`, `eval/`. PTQ 샘플은 `models/ptq`,
   평가 묶음은 `QEXP094_DIR`(K230 0.9.4 평가, 저장소 밖)에서 읽는다.
3. 변환(Pulsar2 6.0-lite, docker amd64):
   작업 디렉터리를 `/data`로 마운트하고 `pulsar2 build --config /data/pulsar2_u16_u8in.json`.
   NPU 코어 하나용(NPU1: 다른 코어는 AI-ISP가 쓴다), U16 전체, 이미지 입력은
   uint8(`input_processors`)이다. 결과 `build/core.axmodel`을 저장소의
   `models/supercombo.axmodel`로 넣고 `models/manifest.sha256`을 갱신하면
   `scripts/upload_to_board.sh`가 보드로 보낸다.

참고: Elu는 Pulsar2 6.0/7.0 모두 U16에서 잘못 컴파일돼서(0.9.4 모델) master 모델로 옮겼다.
보드 NPU 추론은 코어 하나에서 약 15.5 ms다.
