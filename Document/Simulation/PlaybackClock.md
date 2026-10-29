# 배속과 크레인 흔들림

기준일: 2026-10-29. [현재 구현 현황](../Planning/ImplementationStatus.md) · [남은 배속 검증](../Planning/ImplementationRoadmap.md#2-배속과-무관하게-1배속-기준-물리-적용)

월드 시간 간격은 모든 배속에서 1/60초로 고정한다. `UPortSimTimeStep`의
배속은 엔진 tick 사이의 목표 대기 시간(1/60초 ÷ 배속)에만 적용한다.
이전의 `실제 경과 시간 × 배속` 방식은 8배속에서 환경·센서·키네마틱
목표와 Chaos 사이의 갱신 간격까지 늘렸다. 크레인 내부 적분만 나누어도
월드 수준의 갱신 순서와 관측 간격 차이를 없애지는 못한다.

부하가 커지면 물리 시간 간격을 늘리지 않고 실제 달성 배속이 낮아진다.
엔진 tick은 배속에 따라 늘어난다. 후속 성능 수정에서 standalone/패키지
실행의 화면 그리기는 실제 시간 기준 최대 60Hz로 분리했다. Editor PIE는
에디터 엔진을 사용하므로 이 화면 제한의 적용 대상이 아니다. 8배속 달성은 보장하지 않는다.
프레임 지연이나 디버거 정지 뒤에 누적 시간을 몰아서 계산하지 않는다.
와이어의 감쇠·장력·anti-sway 설정은 변경하지 않는다.

`PortSim.Playback.FixedWorldStep` 자동화 테스트는 1/2/4/8/16배속,
배속 전환 및 프레임 지연에서 월드 delta와 시간 증가량을 검사한다.
이는 실제 화면에서 1배속/8배속 와이어 궤적을 비교하는 검증을 대신하지 않는다.

2026-10-29 최초 수정 검증: UE 5.6 Development Editor 빌드 성공,
`PortSim.Playback.FixedWorldStep` 1개 테스트 통과. 화면 궤적 비교 및
전체 터미널의 실제 달성 배속은 측정하지 않았다.

## 후속 성능 개선 — 2026-10-29

고정 시간 간격 도입 후 실제 배속이 설정값보다 낮다는 보고에 따라 동일한
저장 설정·환경 시드로 수정 전후를 측정했다. 시간 간격은 계속 1/60초다.

- 기존 `PrepareKinematicPhysicsFrame`은 큰 시간 간격의 수백 회 Chaos 적분을
  피하려고 매 프레임 전체 액터·컴포넌트를 순회했다. 고정 1/60초 프레임에서는
  이 순회를 생략하고 정상 Chaos 적분을 사용한다. 자유·수면 강체를 임의로
  제외하지 않는다. 0초 및 긴 프레임의 기존 경로는 유지한다.
- tick 대기 기준 시각을 실제 깨어난 시각에서 매번 다시 잡지 않고 이전
  예정 시각에서 이어 간다. 스케줄러 지연이 매 프레임 누적되지 않게 하며,
  지연은 한 tick 범위로 제한하여 긴 정지 이후 대량으로 따라잡지 않는다.
- `UPortSimGameEngine`은 standalone/패키지 화면 그리기를 실제 시간 최대
  60Hz로 제한한다. `PortSim.PresentationHz`로 1~120Hz 범위를 조절할 수 있다.
  물리·센서 tick은 유지하고 와이어·스프레더·화물을 같은 완료 상태에서 그린다.
  와이어만 별도로 갱신을 지연시키지 않는다.

### 측정 결과

조건: `MeasurePlayback.ps1 -PlaybackRate 8 -Seconds 20 -EngineStats`,
환경 시드 `166290927`, null RHI(렌더링 없음), 실행 시 저장된 설정 사용.

| 항목 | 수정 전 | 수정 후 |
|---|---:|---:|
| 실제 측정 시간 | 20.004초 | 20.002초 |
| 처리한 시뮬레이션 시간 | 48.350초 | 82.000초 |
| 실제 달성 배속 | 2.417배 | 4.100배 |
| 물리 프레임 정책 평균 비용 | 2.180622ms | 0.005049ms |
| 완료 운반 수 | 0 | 0 |

이 구간에서 실제 진행 속도는 약 69.6% 개선됐다. **설정값 8배속에는 아직
도달하지 않았다.** 운반 완료가 없는 짧은 측정이며, 대규모 하역 처리량이나
렌더링/Editor PIE 성능을 나타내지 않는다. 화면 제한의 효과도 위 수치로
검증한 것은 아니다. 대시보드 수집은 수정 후에도 20초 중 약 2.884초를
사용했으며 이번 변경에서는 수집 로직을 수정하지 않았다.

원본 로그: `PortSim/Saved/Logs/PlaybackFixed_Before.log`,
`PortSim/Saved/Logs/PlaybackFixed_After.log`.
요약: `PortSim/Saved/PlaybackFixed_Before.json`, `PortSim/Saved/PlaybackFixed_After.json`.

성능 수정 후 UE 5.6 Development Editor 빌드 성공. 회귀 테스트
`PortSim.Playback.FixedWorldStep`, `PortSim.Performance.SensorQueriesWithoutOverlapEvents`
총 2개 통과. 로그: `PortSim/Saved/Logs/PlaybackPerformance_Regression.log`.
테스트 시작 전 초기화 로그에는 `Condition failed` 3건이 있었으며, 두 테스트의
최종 판정은 각각 `Success`였다. 전체 터미널의 배속별
물리 동등성, 실제 와이어 궤적 비교와 장시간 처리량은 여전히 별도 검증 대상이다.

### 렌더링 실행 확인

같은 설정에 `-Render`를 추가한 1280×720 창/D3D12 실행에서는 실제
20.071초 동안 시뮬레이션 0.650초(0.032배), 완료 운반 0건을 기록했다.
로그에는 250건 이상의 PSO 생성 지연과 100~200ms PSO 대기가 있었다.
초기 그래픽 파이프라인 생성이 포함된 결과이므로 정상 상태의 렌더링
처리량으로 해석하지 않는다. 화면 제한만으로 이 초기 지연은 해결되지
않았으며, 워밍업 이후 재측정과 PSO 준비 비용 개선이 남아 있다.

원본: `PortSim/Saved/Logs/PlaybackFixed_Render.log`,
`PortSim/Saved/PlaybackFixed_Render.json`. 렌더링 성능 개선이나 와이어의
시각적 정상 동작이 검증됐다고 선언하지 않는다.
