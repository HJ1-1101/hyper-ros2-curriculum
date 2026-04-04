# HYPER — ROS 2 스터디 클럽 커리큘럼

2~3학년 학생들을 위한 4주 완성 ROS 2 실습 과정입니다. 매주 이론 강의, 실습(In-Class Lab), 과제로 구성됩니다.

**스택:** Ubuntu 24.04 · ROS 2 Jazzy · Python 3 · C++20 · Gazebo · RViz

---

## 전체 일정

| 주차 | 날짜 | 주제 |
|---|---|---|
| 1주차 | 3월 11일 | 생태계, 워크스페이스 & 패키지 |
| 2주차 | 3월 18일 | 토픽, 서비스 & 커스텀 인터페이스 |
| 3주차 | 3월 25일 | URDF & RViz |
| 4주차 | 4월 2일 | Gazebo, Xacro & 파라미터 |

---

## 1주차 — 생태계, 워크스페이스 & 패키지

**핵심 개념:** ROS 2 미들웨어, DDS, 컴퓨테이션 그래프(Computational Graph). `hyper_ws` 생성 및 `src/` 디렉토리 구조 이해. C++과 Python 패키지 생성 (`ros2 pkg create --build-type ament_python`). `package.xml`과 `setup.py`의 역할.

**실습:** 타이머를 사용하여 매초 콘솔에 메시지를 출력하는 C++, Python "Hello World" 노드 작성.

**과제: "파이썬과 C++ 인사시키기"**

`hyper_ws` 안에 두 개의 패키지를 추가하세요:

- `pyhello` — 1초마다 `"Hello World from pyhello!"`를 토픽으로 발행하고 터미널에도 출력
- `cpphello` — `/hello_topic`을 구독하여 `"I heard: 'Hello World from pyhello!'"` 를 터미널에 출력

**제출물:** VS Code 터미널을 split하여 한쪽은 `ros2 run pyhello <노드명>`, 반대쪽은 `ros2 run cpphello <노드명>` 실행 화면.

---

## 2주차 — 토픽, 서비스 & 커스텀 인터페이스

**핵심 개념:**
- **토픽 (Pub/Sub):** 연속적인 데이터 스트림 — "라디오 방송" 모델
- **서비스 (Client/Server):** 요청-응답 — "전화 통화" 모델
- **인터페이스:** 표준 메시지(`std_msgs`, `geometry_msgs`) 사용 및 `package.xml`에 의존성 선언
- `rqt_graph`, `rqt_plot`으로 토픽 시각화
- `rosidl_default_generators`를 통한 커스텀 인터페이스 생성

**실습:** `CustomMsg.msg`와 `CustomSrv.srv`를 직접 작성.

**과제: "스마트 배터리 관리 시스템"**

`hyper_ws` 안에 두 개의 패키지를 생성하세요:

**`my_robot_interfaces`** — 커스텀 메시지와 서비스 정의:
- `BatteryStatus.msg` — 전압, 잔량, 상태 문자열, LED 색상
- `SetLedColor.srv` — 요청: 변경할 색상 / 응답: 성공 여부(boolean)

**`smart_battery_system`** — 두 개의 노드로 구성된 Python 패키지:
- **노드 A:** 전압 5V, 잔량 100%, 상태 `"Normal"`, LED `green`으로 초기화. 1초마다 배터리를 5%씩 감소시키며 `/battery_info`에 발행. 잔량이 20% 미만이 되면 상태를 `"Low"`로 변경.
- **노드 B:** `/battery_info`를 구독하다가 잔량이 20% 미만으로 내려가면 LED를 `red`로 변경하는 서비스 요청 전송.

**제출물:**
- 노드 A, B가 동시에 실행되는 split 터미널 화면
- 두 노드가 표시된 `rqt_graph` 스크린샷 (노드가 안 보이면 상단의 *Hide: Debug* 비활성화)
- `rqt_plot`에서 배터리 잔량이 표시된 그래프

---

## 3주차 — URDF & RViz

**핵심 개념:** 링크(팔, 바퀴)와 조인트(revolute, continuous) 정의. 시뮬레이션에서 로봇이 날아가지 않도록 `<collision>`과 `<inertial>` 속성 추가. 명령어 하나로 RViz와 `joint_state_publisher_gui`를 동시에 실행하는 launch 파일 작성.

**실습:** 간단한 1-DOF 다리 URDF를 다운받아 Gazebo에 스폰하고 `joint_state_publisher_gui`로 움직여보기.

**과제: "나만의 미니 로봇 팔 띄우기"**

`hyper_ws` 안에 `hyper_description` 패키지를 생성하고, 아래 조건을 만족하는 로봇 팔 URDF를 작성하세요:

1. **구조:** 바닥에 고정된 Base를 포함하여 최소 3개의 Link와 2개의 움직이는 Joint(`revolute` 또는 `continuous`)
2. **디자인:** `<material>` 태그로 각 Link의 색상을 다르게 지정 — `cylinder`, `box` 등 기본 도형만 사용해도 무방
3. **Launch 구성:** `ros2 launch hyper_description display.launch.py` 명령어 하나로 RViz와 `joint_state_publisher_gui`가 동시에 실행되도록 설정

**제출물:**
- 작성한 URDF/Xacro 파일 코드 캡처
- GUI 슬라이더로 두 관절이 각각 움직이는 화면 녹화 영상 또는 GIF

---

## 4주차 — Gazebo, Xacro & 파라미터

**핵심 개념:**

- **Gazebo Sim:** 중력 $g \approx 9.81\ \text{m/s}^2$, 마찰, 충돌 등 물리 법칙이 작동하는 가상 세계 — 단순 시각화 도구인 RViz를 넘어선 시뮬레이션 환경
- **Xacro:** `${property}` 변수와 `include` 문을 활용한 URDF 매크로 확장 — 복잡한 모델의 재사용성 극대화
- **SDF (Simulation Description Format):** 물리 엔진 설정, 광원, 지형 등 '환경' 묘사에 특화된 Gazebo 전용 포맷
- **YAML 파라미터:** 제어기 설정, 조인트 이름, Gain 값 등을 코드 수정 없이 관리
- **`ros_gz_bridge`:** ROS 2 DDS와 Gazebo Transport 사이에서 메시지를 실시간으로 통역하는 브릿지

**실습:** 제공된 `mini_pupper` URDF를 Jazzy 환경에 맞게 업데이트하고 RViz에 시각화. Gazebo Component Inspector로 관성 정보 확인 및 Lidar/카메라 센서 데이터 흐름 파악.

**과제: "Pupper의 첫 걸음마 — 보행 제어기 통합"**

배포된 `quadruped_controller` 패키지(Gait 생성 및 IK 연산 포함)와 실습에서 구축한 Gazebo 환경을 연결하여 4족 보행 로봇을 실제로 움직여보세요.

1. **브릿지 설정:** `gazebo.launch.py`를 수정하여 `/clock`, `/scan`, `/tf`, `/camera/image_raw` 토픽이 `ros_gz_bridge`를 통해 ROS 2와 Gazebo 사이에서 정상적으로 공유되도록 설정

2. **제어 루프:**
   - **입력:** `teleop_twist_keyboard`로부터 수신되는 `/cmd_vel` (`Twist`) 명령
   - **로직:** 전진 속도 $v_x$와 회전 속도 $\omega_z$를 기반으로 Trot 보행 패턴을 생성하고, 역운동학($IK$)을 풀어 12개 관절의 목표 각도 산출
   - **출력:** 계산된 관절 각도를 `trajectory_msgs/JointTrajectory` 형식으로 `/leg_controller/joint_trajectory` 토픽에 발행

**제출물:**
- `teleop_twist_keyboard` 조작에 따라 Gazebo 상의 Pupper가 보행하는 화면 녹화 영상
- Teleop → Controller → Bridge → Gazebo 제어 흐름이 명시된 `rqt_graph` 스크린샷
