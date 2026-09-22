# Camera

폰 스테레오 영상으로 객체를 인식하고 객체별 깊이·거리·크기를 추출합니다. RGB·깊이맵·전체 장면 3D 점군·IMU를 통합 뷰어에서 확인할 수 있습니다.

## 실행

저장소 루트에서, 폰을 USB로 연결한 뒤 실행합니다. 기존에 구성된 `venv_ffs` 환경과 폰 카메라 앱을 사용합니다.

```bash
./venv_ffs/bin/python z_phone_stereo.py \
  --physical 2 5 --calib-stereo calib_phone_pair.json \
  --yolo seg --rgbd-viewer --out phone_stereo/runs/my_view
```

- `--yolo seg`: 객체 인식·세그멘테이션 및 객체별 깊이 추출.
- `--rgbd-viewer`: RGB·깊이·3D 점군·IMU 표시.
- `--out`: 실행 결과 저장 폴더.

인식한 객체의 3D 모델을 맵에 추가·누적하는 기능은 포함하지 않습니다. `--object-map` 옵션은 제거되었습니다.

뷰어에서 드래그로 3D 회전, 휠로 확대, RGB/깊이 영상 클릭으로 거리·XYZ를 확인합니다. **F**는 화면 고정, **E**는 현재 프레임 내보내기, **Q**는 종료입니다.

설치·보정·저장 파일 등 자세한 설명은 [폰 스테레오 README](phone_stereo/README.md)를 참고하세요.
