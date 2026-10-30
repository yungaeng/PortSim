# 현재 구현 현황

기준일: 2026-10-30. 현재 작업 트리의 구현과 검증 범위를 정리했다. 배속 기능은 이번 재빌드와 회귀 검사를 반영했고 다른 분야의 검증 범위는 기존 기록을 따른다. 과거 시험 조건과 결과는 각 상세 문서를 따른다.

| 분야 | 코드에서 확인한 구현 | 남은 범위 / 한계 |
|---|---|---|
| 하역·배차 | STS → AGV → RMG → 야드 운송, STS 빈차 선배치, 작업 준비 상태 기반 배차, 도로·야드 접근 예약, 대기량을 고려한 계획 | 처리량은 조건별 측정이 필요하며 목표값을 달성한 것으로 간주하지 않음 |
| 선적 | `ScheduleExport`와 `AdvanceExport`에서 야드 → RMG → AGV → STS → 선박 운송, 크레인 가용성·선박 하부 지지·해당 열 하역 완료 검사 | 하역 수준의 장비 선배치·대기 최적화와 양방향 작업 조정 확장 필요 |
| 화물·야드 | 품목·취급 유형·위험물 모의 그룹·목적항·중량 정보, 크기별 슬롯과 층별 지지·중량 검사, 조건부 20ft 두 개 운송 | 냉동 전력·온도 유지와 실제 위험물 격리표 구현은 별도 범위 |
| 품목별 구역 | `ZoneAccepts`에 일반 1~18, 냉동·냉장 19~21, 인화성 22, 부식성 23 구역 규칙 존재 | 하역·선적·고장 재계획 전체 경로에서 일관성 검증 및 복합 조건 확장 필요 |
| AGV | 경로·교통 예약, LiDAR/FMS 정지, 배차·충전, 타이어 슬립·하중 이동·제동·환경 마찰 계산 | 전체 차량 6DOF 모델과 동일하다는 의미는 아님 |
| STS·RMG | 센서 기반 정렬, 잠금 피드백, 시험 인양, 현수 동역학, 하중·지지면·충돌 인터록 | 센서/기계 설정에는 공개 자료와 시뮬레이션 가정이 함께 포함됨 |
| 고장 복구 | STS/RMG 고장 격리와 조건부 작업 재배정, 수리 후 재검사, 주차 칸의 막힌 빈 AGV 작업 교체 | 화물을 보유한 고장 AGV에서 다른 AGV로 현장 인양·인계하는 기능과는 다름 |
| Reach Stacker | `APortSupportVehicle`에 장비 종류·번호와 외형 구성 존재 | 현장 출동, 센서 기반 인양, 타이어·집게 접촉 물리, 대체 AGV 인계는 추가 구현 대상 |
| 배속 | 통합 터미널의 고정 1/60초 사전 계산·체크포인트 복원·60Hz 목표 재생. 외부 입력 시 미래 폐기와 1배속 전환. 요청 8배속 null RHI 20초 실측 7.638배 | Chaos 복원 제외, 해당 구간은 기존 실시간 경로. 렌더링 60fps·전체 운송 및 장시간 동등성은 미검증 |
| 환경·관제 | 바람·돌풍·선박 운동·강수/마찰, 장비·화물·센서 데이터, 실시간 물리 관측, 고장·작업자 진입 제어 | 날씨는 현재 웹에서 관측 전용 |
| 선박 적재 하중·흔들림 | 기존 환경 기반 선박 운동과 선상 화물 운동 연동 | 컨테이너 무게·적재 위치·선적/하역에 따른 선박 질량·무게중심·하중 분포 변화와 선체 흔들림 연동 구현 필요 |

## 코드 근거

실제 터미널 체크포인트 재생을 연결했다. 환경·센서·제어기·예약·소유권·난수 상태를 복원하고 확정한 결과만 공개한다. 사용자 요청으로 Chaos 내부 복원은 제외한다. [현재 구현과 검증](../Simulation/TerminalReplay.md). 현수체 비동기 시제품은 [별도 이력](../Simulation/ShortLookaheadPrototype.md)이다.

- 운송·배차: [ExportOperations.cpp](../../PortSim/Source/PortSim/ExportOperations.cpp), [PortSiteLogistics.cpp](../../PortSim/Source/PortSim/PortSiteLogistics.cpp), [AGVDispatchRecovery.cpp](../../PortSim/Source/PortSim/AGVDispatchRecovery.cpp)
- 화물·적재: [CargoInformation.h](../../PortSim/Source/PortSim/CargoInformation.h), [YardStackingPolicy.h](../../PortSim/Source/PortSim/YardStackingPolicy.h), [ShipStowagePlanning.cpp](../../PortSim/Source/PortSim/ShipStowagePlanning.cpp)
- 고장·지원 장비: [EquipmentRecovery.cpp](../../PortSim/Source/PortSim/EquipmentRecovery.cpp), [PortSupportVehicle.h](../../PortSim/Source/PortSim/PortSupportVehicle.h), [PortSupportVehicle.cpp](../../PortSim/Source/PortSim/PortSupportVehicle.cpp)
- 시간·물리: [PortSimTimeStep.cpp](../../PortSim/Source/PortSim/PortSimTimeStep.cpp), [SimulationControls.cpp](../../PortSim/Source/PortSim/SimulationControls.cpp), [AGVDynamics.cpp](../../PortSim/Source/PortSim/AGVDynamics.cpp)

## 상세 문서

2026-10-29 배속 후속 수정은 별도로 빌드·회귀 테스트를 실행했다. 2026-10-30에는 고배속 대시보드 수집 주기를 비용에 따라 1~2초로 조절하고 중복 조회를 줄였다. 동일 조건의 null RHI 20초 비교에서 7.449배 → 7.702배를 측정했다. [배속과 크레인 흔들림](../Simulation/PlaybackClock.md)에 결과와 미검증 범위를 기록했다. 이 결과는 전체 기능의 통합 검증을 뜻하지 않는다.

[하역 처리량](../Operations/DischargeThroughput.md) · [야드 적재와 묶음 운송](../Yard/StackingAndPairedTransport.md) · [고장 복구](../Safety/EquipmentFailureRecovery.md) · [배속 성능](../Simulation/PlaybackPerformance.md) · [실시간 물리 관제](../Dashboard/LivePhysics.md)

기존 문서의 날짜별 시험 기록은 당시 구현의 이력이다. 특히 야드 문서의 “선박 적재 계획 미리보기만 제공” 설명은 후속 선적 코드보다 이전 상태이므로 현재 선적 지원 여부는 위 코드 근거를 우선한다.

다음 개발 범위: [향후 구현 계획](ImplementationRoadmap.md).

주의: 현재 대시보드·에디터의 특수 구역 표시는 참조 이미지 기반 배치이며, 위 야드 배정 정책의 구역 번호와 다르다. 표시 변경은 화물 배정을 바꾸지 않으므로 향후 품목별 배정 작업에서 표시와 실제 정책을 함께 맞춰야 한다.
