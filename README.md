### 리눅스 도커 설치
```
sudo apt-get update
sudo apt-get install docker.io
sudo ln -sf /usr/bin/docker.io /usr/local/bin/docker
sudo usermod -aG docker user
newgrp docker
sudo wget -qO /etc/apt/keyrings/nvidia-container-toolkit.asc https://nvidia.github.io/libnvidia-container/gpgkey
echo "deb [signed-by=/etc/apt/keyrings/nvidia-container-toolkit.asc] https://nvidia.github.io/libnvidia-container/stable/deb/amd64 /" \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

```


### 도커 이미지 빌드

```
docker build -t  easy-collector:latest .
```

### 도커 최초 빌드 시
```
cd ~/src/ui
npm install

cd ~/python_pkgs
bash install.sh
```


### 도커 컨테이너 실행

```
bash ./run.sh
```


### 도터 컨테이너 접속

```
docker exec -it easy_collector_container /bin/bash
```


### 도터 컨테이너 탈출

```
exit
```


### torch upgrade
```
pip install --upgrade torch
```

### 서버 시작
```
cd ~/src
python3 -m backend.api.app
```


### 인터페이스 시작
```
cd ~/src/ui
npm run dev
```


## For User
### 터미널1을 켜고 (docker)
```
bash ./run.sh
```


### 터미널2를 켜고 (backend)
```
docker exec -it easy_collector_container /bin/bash
```
```
cd ~/src
python3 -m backend.api.app
```

### 터미널3을 켜고 (ui)
```
docker exec -it easy_collector_container /bin/bash
```
```
cd ~/src/ui
npm run dev
```


### 데이터베이스에 column 추가 시 해야할 일
1. migration
2. model에 추가
3. 프론트엔트에 추가
4. api에 추가
