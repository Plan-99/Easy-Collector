from licensing.models import *
from licensing.methods import Key, Helpers
import sys

# === [설정 구간] Cryptolens 대시보드에서 가져와야 함 ===
# 1. Product ID (제품 번호)
PRODUCT_ID = 32008  
# 2. RSA Public Key (보안 키)
RSA_PUB_KEY = "<RSAKeyValue><Modulus>sr0fRk+t/ddNZdGOnMzNoA+IVlGwX0GwQHHKgVEueazhFtZZGdEcnUO8Gtwi1xv0XgM6B762RULQ/1xE1fPi1/RJBzlyIlpH/8ybrbH6S0cvu0TpHHY+pv3SYZduAcHlazKv3N7Fi3A6lyGwJQEj1UsrbINFhFiXZHGXiaggY+Fwr0CJUhZ2wjGyjCWOk1UdhXjd6eMzd98JJ4DJC0n1QnFbaawdWureXdLLDRn47bu322r9iokbpNReQVpBDc9fHx0kIouQbSGuYYkkyjAxIIuaca0meH8XPa7laodplYzRsPL+xZhEAouTII2M8gg/lwpMXepDMv/WiKoQNuK4/Q==</Modulus><Exponent>AQAB</Exponent></RSAKeyValue>"
# 3. Access Token (권한 토큰) - 'Activate' 권한이 있어야 함
ACCESS_TOKEN = "WyIxMTY1MjUwNzkiLCJyQ1VLeTN2WERVNHpyNmRmQ2FwUlFDVlR6RGFFRHFVcGhpU3ozUE91Il0="

def verify_license(key_string):
    """
    라이선스 키를 입력받아 유효성을 검사하는 함수
    """
    try:
        # 1. 현재 기기의 고유 지문(Machine Code) 생성
        machine_code = Helpers.GetMachineCode()

        # 2. 서버에 검증 요청 (함수명 소문자 주의: activate)
        # 반환값은 (LicenseKeyObject, MessageString) 형태의 튜플입니다.
        result = Key.activate(
            token=ACCESS_TOKEN,
            rsa_pub_key=RSA_PUB_KEY,
            product_id=PRODUCT_ID,
            key=key_string,
            machine_code=machine_code
        )

        # 3. 결과 확인
        # result[0]이 None이면 인증 실패
        # Helpers.IsOnRightMachine으로 기기 검증까지 해야 확실함
        if result[0] is None or not Helpers.IsOnRightMachine(result[0]):
            # 실패 이유를 출력 (디버깅용)
            print(f"[Fail] 인증 실패 사유: {result[1]}")
            return False

        # 성공 시
        print(f"[Success] 인증 성공! (만료일: {result[0].expires})")
        return True

    except Exception as e:
        print(f"[Error] 라이선스 인증 중 오류 발생: {e}")
        return False

def get_machine_fingerprint():
    """사용자가 내 기기 ID를 물어볼 때 알려주는 용도"""
    return Helpers.GetMachineCode()