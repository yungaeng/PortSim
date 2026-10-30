# 문서 안내

먼저 [현재 구현 현황](Planning/ImplementationStatus.md)과 [향후 구현 계획](Planning/ImplementationRoadmap.md)을 확인하세요.

2026-10-29에 주제별로 정리했습니다. 기존 기술 문서는 구현·검증 이력을 보존하며, 날짜가 있는 과거 결과는 현재 전체 기능의 검증 결과와 구분합니다. 실행에 사용되는 STS 기준 JSON 경로는 유지했습니다.

## Planning — 기획·구현 현황·향후 계획

- [향후 구현 계획](<Planning/ImplementationRoadmap.md>)
- [현재 구현 현황](<Planning/ImplementationStatus.md>)
- [항만_자동화_중간발표_수정본.pdf](<Planning/Presentations/항만_자동화_중간발표_수정본.pdf>)
- [항만_컨테이너_자동화_시뮬레이터_기획서.pdf](<Planning/Presentations/항만_컨테이너_자동화_시뮬레이터_기획서.pdf>)

## Operations — 하역·선적·입항 운용

- [STS 하역 처리량 진단과 개선](<Operations/DischargeThroughput.md>)
- [입항 선박의 실제 초기 적재 배치](<Operations/ShipArrivalStowage.md>)

## AGV — AGV 배차·교통·센서·물리

- [자동화 컨테이너 터미널에서의 AGV 충돌 방지 및 교착 해결 방안.pdf](<AGV/자동화 컨테이너 터미널에서의 AGV 충돌 방지 및 교착 해결 방안.pdf>)
- [AGV 돌발상황 통합 (2026-10-24)](<AGV/AGV_Emergency.md>)
- [AGV 노면 슬립·CoG 안정성 (2026-10-22)](<AGV/AGV_Physics.md>)
- [AGV 배터리와 충전소](<AGV/Charging.md>)
- [AGV 배차·컨테이너·목적 야드 계획](<AGV/DispatchAndContainerPlanning.md>)
- [ga-zpmc-qc8-12.pdf](<AGV/ga-zpmc-qc8-12.pdf>)
- [배 반대쪽 도로의 FMS 대기 수정](<AGV/InlandFMSQueues.md>)
- [적재 AGV 교착 복구와 주차 방식 비교](<AGV/LoadedTrafficRecoveryAndParkingComparison.md>)
- [upstream main AGV 대기소 병합 (2026-10-15)](<AGV/MainDepotMerge.md>)
- [STS 픽업 차량의 도로 진입 우선순위](<AGV/PickupRoadPriority.md>)
- [실행 가능한 작업 우선 배차](<AGV/ReadyWorkDispatch.md>)
- [옛 AGV 대기 공간의 운반 차선 전환](<AGV/ReusedParkingApron.md>)
- [AGV 위치추정·FMS·LiDAR 런타임 적용](<AGV/RuntimeSensorApplication.md>)

## STS — STS 센서·동역학·참고 자료

- [STS 현수체·구동·장착 센서 모델](<STS/DynamicsModel.md>)
- [STS 코드 확장 계획과 추가 적용 기능](<STS/ImplementationPlan.md>)
- [부산 신항 7부두(DGT)](<STS/LegacyDGTLayout.md>)
- [STS 도면·센서 자료 분석 및 프로젝트 적용 명세](<STS/README.md>)
- [DGT_Hyundai_STS_Sensor_Installation_Reference.pdf](<STS/References/DGT/DGT_Hyundai_STS_Sensor_Installation_Reference.pdf>)
- [DGT_STS_Sensor_Coordinates.csv](<STS/References/DGT/DGT_STS_Sensor_Coordinates.csv>)
- [SOURCE_PDF_LINKS.txt](<STS/References/DGT/SOURCE_PDF_LINKS.txt>)
- [STS_Sensor_Model_Specifications.csv](<STS/References/DGT/STS_Sensor_Model_Specifications.csv>)
- [ga-zpmc-qc8-12.pdf](<STS/References/ga-zpmc-qc8-12.pdf>)
- [STS_Crane_Sensor_Layout_AI_Reference.txt](<STS/References/STS_Crane_Sensor_Layout_AI_Reference.txt>)
- [STS 레퍼런스의 런타임 적용](<STS/RuntimeApplication.md>)
- [센서 기반 STS 픽업](<STS/SensorPickup.md>)
- [공개 센서 자료 적용 · 2026-09-24](<STS/SensorSourceApplication.md>)
- [SensorSourceReview.json](<STS/SensorSourceReview.json>)
- [STS_ReferenceData.json](<STS/STS_ReferenceData.json>)

## RMG — RMG 센서 제어·참고 자료

- [RMG 참고 이미지](<RMG/References/addendum_to_operating_instructions_aos104_rtg_object_detection_systems_en_im0065347.pdf>)
- [RMG 참고 이미지](<RMG/References/IDWjE8LWfZfVGGAqWib1XZN3llzwt-k8L8O24Lw4qPv6TEDX6MfHi5cXWXigPI0y6PYakaQKHA5qey4F6gSdNY272g0SEay6S2TQ41r1rWup1FvmIj5UpfGmJ3PluDvUyCAXry4Qq0X9VD6n-D_-F0D3TLBx2vEGGxiQv32QxkdtrvnLs7Sj6.jpg>)
- [konecranes_rmg_tech_document_final_nov_2023.pdf](<RMG/References/konecranes_rmg_tech_document_final_nov_2023.pdf>)
- [Upgrade_Container_Weighing_LSS.pdf](<RMG/References/Upgrade_Container_Weighing_LSS.pdf>)
- [Yard-Automation-Brochure-revision2019-12-11.pdf](<RMG/References/Yard-Automation-Brochure-revision2019-12-11.pdf>)
- [RMG 센서 기반 운반](<RMG/SensorControl.md>)

## Yard — 야드 배정·적재·묶음 운송

- [물품 기반 야드 적재와 20ft 묶음 운송](<Yard/StackingAndPairedTransport.md>)

## Safety — 장비 고장 복구·작업자 안전

- [센서 고장 격리와 작업 재배정](<Safety/EquipmentFailureRecovery.md>)
- [웹 작업자 돌발 진입](<Safety/WorkerIntrusion.md>)

## Simulation — 배속·시뮬레이션 성능

- [터미널 체크포인트 배속 재생 — Chaos 제외 범위·검증·사용법](<Simulation/TerminalReplay.md>)
- [짧은 구간 사전 계산 시제품 — 초기 구현 이력](<Simulation/ShortLookaheadPrototype.md>)
- [배속과 크레인 흔들림 — 고정 월드 시간 간격 수정](<Simulation/PlaybackClock.md>)
- [배속 성능 회귀와 수정](<Simulation/PlaybackPerformance.md>)

## Environment — 날씨·마찰·운전 조건

- [날씨에 따른 장비 마찰 모델](<Environment/Weather_Friction.md>)
- [Weather operation hold](<Environment/WeatherOperations.md>)

## Dashboard — 웹 관제·실시간 물리·터미널 오버뷰

- [실시간 AGV 물리와 돌발상황](<Dashboard/LivePhysics.md>)
- [웹 터미널 오버뷰 (2026-10-24)](<Dashboard/TerminalOverview.md>)
- [웹 장비 고장 제어와 영향 관측](<Dashboard/WebEquipmentControl.md>)

## Terminal — 터미널 배치 참고 이미지

- [Bird’s eye view.jpg](<Terminal/DGT/Bird’s eye view.jpg>)
- [floor plan1.png](<Terminal/DGT/floor plan1.png>)
- [floor plan2.jpg](<Terminal/DGT/floor plan2.jpg>)
- [Satellite view1.png](<Terminal/DGT/Satellite view1.png>)
- [Satellite view2.png](<Terminal/DGT/Satellite view2.png>)

