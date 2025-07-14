DATABASES = {
    'default': 'sqlite',  # 기본으로 사용할 DB 설정 이름

    'sqlite': {
        'driver': 'sqlite',
        # 프로젝트 폴더에 'my_database.sqlite' 라는 파일로 DB가 생성됩니다.
        'database': 'main.db',
        'prefix': ''
    }
}