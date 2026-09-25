# 공개 센서 자료 적용 · 2026-09-24

원본 4개 파일은 `References/DGT/`에 변경 없이 보관한다. `SensorSourceReview.json`에는 두 CSV의 모든 행을 보관한다. 현재 엔진은 ZPMC QC8–12 형상/운전 레퍼런스를 사용하며, 새 현대 DGT 자료는 센서 제품 사양·설치 원칙을 보강한다. 센서 제품의 공개 사양 확인과 DGT 실장 여부 확인은 별개다.

## 적용 결과

| 자료 CSV ID | 기존 프로젝트 ID / key | 변경 |
|---|---|---|
| S01 | S02 / boom_collision_lidar | SICK AOS502 STS의 LMS511-10100S04 2대. 190°·명목 80m. 움직이는 트롤리에서 붐을 따르는 gantry frame으로 이동. X=-40.25, Y=±7.3, Z=44 m. |
| S02 | S03 / trolley_encoder | KH53 1700m 옵션, 0.1mm 계측 분해능, 6.6m/s 초과 관측 무효화. 헤드 간격 25mm를 로컬 +Y로 채택. |
| S03 | S05 / twistlock_load | 트위스트록당 센서 1개 근거 추가. 기존 코너 좌표/측정 범위는 가정 유지. |
| S04 | S06 / twistlock_state, S07 / landed | 기존 4코너 배치에 제조사 원칙 연결. 모델별 정확한 기구 좌표는 미확정. |
| S05 | S08 / spreader_camera | Hawkeye 중앙 하부 패턴을 선택해 [0,0,-0.3]m, 하방 배치. 최대 4대 중 1대 구성. FOV/거리 가정 유지. |
| S06 | S09 / ttds | 구형 38mm 간극·425/730mm 치수는 참조값으로 보존. 현재 스프레더와 동일 모델임이 확인되지 않아 위치/임계값으로 미적용. |
| S07 | S01 / wind | 자료에도 모델/좌표가 없어 기존 가정 유지. |
| S08 | S12 / agv_position_lidar | 인계 구역 양측 설계 2대로 확장. [32,±8,20]m. LMS511 적용 사례와 프로젝트 설계값 구분. AOS 전용 사양을 AGV 모델 확정값으로 복사하지 않음. |

기존 프로젝트 S04 권상 엔코더, S10 길이/간극 LiDAR, S11 신축 엔코더, S13 갠트리 충돌, S14 선박 프로파일은 새 자료에 해당 센서의 확정 모델/수치가 없어 설정을 유지한다. 센서 번호가 다르므로 CSV 행 번호로 자동 덮어쓰지 않는다.

## 좌표와 단위

자료 좌표계는 해측 레일을 원점으로 +X 해측, 엔진은 크레인 원점의 +X 육측이다. 동일 좌표로 복사하지 않는다. 현재 붐 중앙 파생식은 `waterside_rail_x(-8.5) - active_outreach(63.5)/2 = -40.25m`. 자료의 DGT 71.5/2=35.75m는 다른 크레인에서의 파생 예로 보존한다. Y=±7.3, Z=44m와 하방 5° 기울기/스캔면은 현 모델에 맞춘 설계 가정이다. 실물 설치 XYZ가 아니다.

KH53의 0..1700m는 센서의 측정 길이 옵션이다. 엔진의 음수 트롤리 X를 허용 범위로 직접 복사하지 않고 `장치값 = round((엔진X - 영점)/분해능) × 분해능`, 영점=-72m로 변환한다. 축 운전 한계는 기존 레퍼런스 값을 유지한다. 반복성 0.3mm는 정확도나 균등 난수 범위가 아니므로 메타데이터로 보존한다. 25mm의 N축을 모델 +Y로 놓은 것은 가정이며 측정요소 전체 레일은 모델링하지 않는다.

## 제품 사양과 진단 범위

- AOS 80m는 명목 거리, 10% 반사율 물체에는 38m. 38m를 별도 보존하며 현재 엔진에는 반사율 모델이 없다.
- 제조사 기본 정지 필드 3.5×50m, 경고 필드 7.5×50m, 본체 이격 ≥0.5m는 참조 메타데이터다. 실제 제동거리 교정 및 직사각 필드 자동 정지는 이번 설정 교체에 포함하지 않는다.
- 광선은 스캐너마다 9개, 실제 시간 1Hz의 가시성 충돌 진단이다. 전체 3D LiDAR/카메라 영상/OCR/TTDS 인식 기능으로 해석하면 안 된다.
- 웹은 제품 사양 VERIFIED_PRODUCT, 배치 원칙 VERIFIED_LAYOUT_ONLY, 적용 사례 APPLICATION_ONLY, 좌표 DERIVED/ASSUMED를 구분하고 원문 링크를 제공한다. 현재 14종, 마커 25개(붐/AGV 각 1대 추가)다.

## 직접 검토한 주요 원문

- [SICK AOS502 STS 설치·운영 매뉴얼](https://www.sick.com/media/docs/6/46/346/addendum_to_operating_instructions_aos502_sts_objektdetektionssysteme_en_im0065346.pdf): pp.11,16,20–23. 제공된 cdn URL은 조회 실패하여 같은 파일의 sick.com URL 확인.
- [SICK KH53 제품자료](https://www.sick.com/media/docs/0/00/600/product_information_kh53_linear_encoders_en_im0011600.pdf): pp.2–4. 표준형과 Advanced를 구분하고 표준 KH53 옵션 채택.
- [Bromma Load Sensing](https://bromma.com/wp-content/uploads/2019/03/Upgrade_Container_Weighing_LSS.pdf): 코너별 하중·트위스트록당 센서 1개.
- [Bromma Hawkeye](https://bromma.com/wp-content/uploads/2025/11/Hawkeye_Final-Web-1.pdf): p.3 중앙/양측/코너 배치, 최대 4대.
- [Bromma STS45E G2](https://bromma.com/products/sts45e-g2/): 51t, twin 2×32.5t, 자중 12.6t, 신축 약18/21초 확인. DGT 실장 근거가 없어 현재 질량/정격에 미적용.
- [DGT 터미널 특징](https://www.dgtbusan.com/DGT/terminal/feature): 53m lifting height / 72.5m outreach. 제공자료 59m/71.5m와 불일치하므로 ZPMC 형상에 덮어쓰지 않음.
- 나머지 제공 링크도 접근을 시도했다. 부산대 첨부는 PDF 본문으로 반환되지 않았으며 Bromma 통합 브로슈어는 웹 도구 용량 제한으로 열리지 않았다. 두 자료에 의존하는 새 확정값은 적용하지 않았다. 현대 일반 브로슈어·현대삼호 소개·일본 자료는 DGT 센서 as-built XYZ의 증거로 사용하지 않았다.

## 운전 데이터 적용

`STS_Simulation.json`은 엔진 시작 시 로드된다. 에디터에서 진행 중인 PIE를 종료하고 새 빌드로 다시 실행한다. 웹은 실제 실행에 사용한 프로파일 스냅샷을 우선 표시하므로 새 설정 파일만 저장해도 과거 실행의 값이 변경되지는 않는다.

KH53 원문 치수 교정: p.11의 표준형 N=25±10mm와 p.12의 Advanced N=55±20mm를 구분했다. CSV의 55mm 권장 예는 현재 선택한 표준형(0.3mm 반복성/1700m 옵션)과 맞지 않아 25mm로 적용했다. 원본 CSV는 그대로 보관한다.

## 검증 결과

- UE 5.6 프로젝트 빌드 통과.
- STS 프로파일·인터록 및 서스펜션·구동 자동 테스트 통과.
- 통합 정상 운반 테스트: STS 9대의 18건 운반 완료, 마지막 야드 배치까지 시뮬레이션 시간 1214초. 이 값은 인간 운용 대비 절감률 검증 결과가 아니다.
- 웹 텔레메트리 검증 및 시간 비교 계산 테스트 6개 통과. 센서 화면에서 25mm 설치 간격, 양측 스캐너의 탐지 광선, 사양 출처와 원문 링크 표시 확인.
