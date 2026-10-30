# 짧은 구간 사전 계산 시제품

> 이 문서는 최초 시제품과 중간 구현의 이력이다. 이후 Chaos를 제외한 실제 터미널 체크포인트 재생을 연결했다. 현재 상태는 [터미널 체크포인트 배속 재생](TerminalReplay.md)을 따른다. 아래 미구현·미연결 표현은 당시 상태다.

## 2026-10-30 외부 입력과 백그라운드 계산

실제 터미널의 환경 설정 setter(바람·파랑·비·안개·눈·얼음·노면·마찰)와
승인된 웹 장비 명령은 실행 시계에 외부 입력을 알린다. 요청한 배속은 보존하고
실제 tick 대기는 1배속으로 바뀐다. 마지막 입력 후 실제 시간 2초가 지나면 요청 배속으로
복귀한다. 자연적인 돌풍·파랑·노면 누적은 외부 입력으로 처리하지 않는다.
HUD와 웹 스냅샷에는 요청/적용 배속을 구분한다. 직접 UPROPERTY를 수정하는 에디터
입력이나 모든 수동 장비 제어 경로까지 감지하는 기능은 아직 포함하지 않는다.

현수체 시제품은 `SetBackgroundPrediction(true)`를 호출하면 복제한 수치 상태와 입력만
작업 스레드로 보내 미래를 계산한다. 한 번에 한 작업, 최대 120단계로 제한하며
완료 결과의 입력 세대와 시작 시각이 현재 미래 버퍼 끝과 일치할 때만 받는다.
입력/초기화 후 늦게 끝난 이전 계산은 버린다. 작업은 UObject나 시제품 자체를 참조하지
않으므로 객체 소멸 뒤에도 접근하지 않는다. 실행 중인 수치 작업은 제한된 구간까지
끝나지만 결과는 무효화된다. 기본 동기 모드는 기존 테스트와 호출자 호환을 위해 유지한다.

**전체 터미널 사전 계산/60fps 버퍼 재생은 아직 미구현이다.** 실제 터미널의 2초 후 복귀는
현재 기존 고정 단계 실행으로 복귀하며, 사전 계산 버퍼 준비 완료를 기다리는 동작이 아니다.
기존 standalone 최대 60Hz 화면 제한은 유지되지만 이를 녹화 결과 재생으로 표현하지 않는다.
전체 연결에는 Chaos·제어기·센서·예약·소유권·난수의 복원 및 미래 출력 격리가 필요하다.
아래 2026-10-29 기록은 최초 시제품 이력이다.

검증: UE 5.6 Development Editor 빌드 성공. `PortSim.Playback` 3개 및
`PortSim.Dashboard.EnvironmentObservation` 1개, 총 4개 통과.
현수체 검사에는 비동기 실행 중 입력 변경 후 이전 세대 결과 폐기와
새 바람으로 1배속 기준 9단계 계산한 상태의 일치(각도·각속도 허용 오차 1e-12)를 추가했다.
실행 시계 검사에는 반복 입력의 대기 연장, 요청 배속 복귀, 입력 직후 1/60초 유지를 추가했다.
로그: `PortSim/Saved/Logs/PlaybackInput_Regression.log`.
엔진 초기화의 기존 `Condition failed` 3건과 별개로 테스트 최종 결과는 모두 Success였다.

2026-10-29: 최초 소스 저장 후 사용자 요청으로 UE 5.6 Development Editor
빌드를 수행했고 **성공했다** (`BuildPortSim.ps1`, `PortSimEditor Win64 Development`).
자동화 테스트와 시뮬레이션 실행은 수행하지 않았다. 빌드 성공은 기능 검증 완료를 뜻하지 않는다.

## 구현 범위

`FSuspensionLookahead`는 기존 `FSTSSuspension`의 복제 상태를 사용해 기본
0.5초(30단계)의 미래를 계산한다. 물리 간격은 1/60초이며 기존 현수체
솔버를 사용한다. 미래 계산이 현재 표시 상태를 변경하지 않는다.
전체 터미널·Chaos 월드 복원이나 실제 화면 연결은 아직 구현하지 않았다.
현재 앱을 실행해도 이 기능이 자동으로 활성화되지는 않는다.

- `Reset`: 초기 상태와 유효한 동역학 입력을 설정하고 준비 모드로 진입한다.
- `SetRequestedRate`: 요청 배속을 1~16배 범위로 지정한다.
- `Tick`: 실제 경과 시간을 받아 준비/실시간/재생 상태를 진행한다.
  호출당 미래 계산 예산 기본값은 8단계이고 버퍼 길이는 최대 2초로 제한한다.
- `SubmitInput`: 제어 이벤트를 받은 호출자가 새 입력 전체를 전달한다.
  미래 버퍼와 재생 잔여 시간을 폐기하고 현재 표시 상태를 보존한 채
  실시간 1배속으로 전환한다. 반복 이벤트도 무입력 대기 시간을 다시 시작한다.
- 기본 2초 동안 새 입력이 없으면 다시 준비 모드로 진입한다. 버퍼를 채운 뒤
  원래 요청 배속으로 재생한다. 준비 중에는 표시 시간이 정지한다.
- 버퍼가 부족하면 준비 대기로 전환한다. 준비 시간이나 긴 프레임 정지의
  지연을 누적해서 나중에 몰아서 재생하지 않는다.
- 미래 계산 중 발생한 현수체 고장은 해당 미래 상태가 재생될 때 확정한다.
  고장 이후의 상태는 더 계산하지 않는다.

## 사용 흐름

```cpp
FSuspensionLookahead Preview;
FSuspensionLookaheadInput Input;
Input.Dynamics = Profile.Dynamics;
Input.PowerW = Profile.HoistPowerW;
Preview.SetRequestedRate(8);
Preview.Reset(InitialSuspension, Input); // 기본 0.5초 미래 구간

// 표시 루프에서 실제 경과 시간을 전달한다. 배속을 곱한 시간을 전달하면 안 된다.
Preview.Tick(WallDeltaSeconds);
const FSTSSuspension& Visible = Preview.DisplayedState();

// 사용자 제어 이벤트가 발생했을 때:
Input.Dynamics.Wind = NewLocalWindMps;
Preview.SubmitInput(Input);
```

이 코드는 별도 미리보기 모델용이다. 실제 크레인의 `SuspensionState`만
이 결과로 교체하면 제어기·센서·화물 상태와 어긋나므로 그대로 연결하지 않는다.

## 남은 연결과 한계

현재 미래 구간에서는 입력을 일정하게 유지한다. 바람·길이·가속도·질량 등의
시간 변화, 센서 피드백과 작업 상태 전이를 미래 모델에 포함하는 작업이 남아 있다.
환경 슬라이더·고장·작업 변경의 자동 감지 연결도 아직 없다. 호출자가 모든
영향 있는 입력에 대해 `SubmitInput`을 호출해야 한다.

예측은 호출 스레드에서 제한된 단계만 동기 실행한다. 백그라운드 계산이나
별도 월드/프로세스를 사용하지 않으며, 연산량 감소나 8배속 달성을 검증하지 않았다.
정확한 상태 복원이 가능한 현수체 수치 모델부터 전환 방식을 시험하는 단계다.
실제 화면·대시보드 연결 전에는 환경·센서·작업·예약·소유권·난수 및 Chaos
상태를 포함한 체크포인트와 외부 이벤트 확정 정책이 필요하다.

## 검증 준비

`PortSim.Playback.ShortSuspensionLookahead` 자동화 테스트 소스를 추가했다.
사전 계산의 표시 상태 격리, 8배속 재생과 동일 물리 단계의 일치, 풍속 변경 시
현재 상태에서 1배속 재개, 무입력 후 복귀, 버퍼 소진 및 반복 입력을 검사한다.
**빌드는 성공했으나 테스트는 실행하지 않았으므로 통과 여부는 미확인이다.**

관련 소스: [SuspensionLookahead.h](../../PortSim/Source/PortSim/SuspensionLookahead.h),
[SuspensionLookahead.cpp](../../PortSim/Source/PortSim/SuspensionLookahead.cpp),
[SuspensionLookaheadTests.cpp](../../PortSim/Source/PortSim/SuspensionLookaheadTests.cpp).
