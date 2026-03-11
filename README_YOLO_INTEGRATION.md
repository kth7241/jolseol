# YOLO 통합 버전 안내

이 압축은 **기존 `jolseol_gpt_mvp` 프로젝트에 YOLO ONNX detector를 연결한 버전**입니다.

## 바뀐 점
- `include/detect/onnx_yolo_detector.hpp`
- `src/detect/onnx_yolo_detector.cpp`
- `include/app/pipeline.hpp`
- `src/app/pipeline.cpp`
- `src/main.cpp`
- `models/yolov8n.onnx` 추가

## 동작 방식
- `opt.yolo_onnx_path`가 비어 있지 않으면 **YOLO ONNX detector 사용**
- 모델 로드 실패 시 **기존 stub detector로 자동 fallback**

## 기본 모델 경로
- `models/yolov8n.onnx`

## Jetson 빌드
```bash
cd ~/jolseol_gpt_mvp_yolo
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

## Jetson 실행
```bash
./traffic_monitor
```

## 참고
- 기본 CSI 파이프라인은 `src/main.cpp`에 들어 있습니다.
- 이미 단계 A에서 성공한 GStreamer 문자열이 있으면 `src/main.cpp`의 `opt.cam_pipeline` 부분을 그 문자열로 유지해서 쓰는 것이 가장 안전합니다.
- 현재 detector는 COCO 기준 `car / motorcycle / bus / truck` 클래스만 남깁니다.
- 현재는 **YOLO ONNX + OpenCV DNN** 방식입니다. FPS가 부족하면 다음 단계에서 TensorRT로 옮기면 됩니다.
