### 도커 이미지 빌드

```
docker build -t easy-collector:latest .
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
