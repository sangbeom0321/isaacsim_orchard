# Isaac Sim Docker Setup

이 디렉토리에는 Isaac Sim 프로젝트를 Docker 컨테이너에서 빌드하고 실행하기 위한 파일들이 포함되어 있습니다.

## 파일 설명

- `Dockerfile`: Isaac Sim 기반 커스텀 이미지 빌드용
- `docker-compose.yml`: 컨테이너 오케스트레이션 및 실행 설정
- `.dockerignore`: Docker 빌드 시 제외할 파일들

## 사용 방법

### 1. Docker Compose로 빌드 및 실행

```bash
# 이미지 빌드 및 컨테이너 실행
docker-compose up --build

# 백그라운드에서 실행
docker-compose up -d --build

# 특정 서비스만 실행 (ROS2 브릿지 포함)
docker-compose --profile ros2 up --build
```

### 2. 컨테이너 접속

```bash
# 실행 중인 컨테이너에 접속
docker-compose exec isaac_sim bash

# 또는 직접 실행
docker exec -it kimm_isaac_sim bash
```

### 3. Isaac Sim 실행

컨테이너 내부에서:

```bash
# Isaac Sim GUI 모드로 실행
./runheadless.sh -v

# 헤드리스 모드로 실행 (스크립트 실행용)
./runheadless.sh --no-window
```

### 4. 캐시 디렉토리 설정

호스트에서 캐시 디렉토리를 미리 생성하세요:

```bash
mkdir -p ~/isaac_sim_cache/docker_isaac-sim/{kit/cache/Kit,cache/{ov,pip,glcache,computecache},logs,data,documents}
```

## 환경 변수 설정

- `ROS_DOMAIN_ID`: ROS2 도메인 ID (기본값: 0)
- `DISPLAY`: X11 디스플레이 포워딩용
- `USER`: 호스트 사용자 이름

## 볼륨 마운트

- 프로젝트 소스코드: `/workspace`
- Isaac Sim 캐시: `~/.cache/ov`, `~/.cache/pip` 등
- X11 소켓: `/tmp/.X11-unix`
- 사용자 데이터: `/root/mounted_folder`

## 문제 해결

### GPU 관련 문제
```bash
# NVIDIA Container Toolkit 설치 확인
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

### X11 포워딩 문제
```bash
# 호스트에서 X11 포워딩 허용
xhost +local:docker
```

### 권한 문제
```bash
# 캐시 디렉토리 권한 설정
sudo chown -R $USER:$USER ~/isaac_sim_cache/
```

## 기존 Docker Run 명령어와의 차이점

- Dockerfile을 통해 이미지 커스터마이징 가능
- Docker Compose로 다중 컨테이너 관리
- 환경별 설정 분리 가능
- 재현성 향상
