# 짧은 구간 사전 계산 시제품

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
