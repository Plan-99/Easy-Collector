# Easy Trainer Homepage (Next.js)

Next.js 기반 풀스택 랜딩 페이지입니다. Google OAuth 로그인, 라이선스 키 관리, 요금제 플랜을 지원합니다.

## 기술 스택

- **Next.js 16** (App Router, Turbopack)
- **NextAuth.js v5** — Google OAuth
- **Prisma 7** + SQLite (→ Supabase로 전환 가능)
- **Tailwind CSS v4**
- **Resend** — 이메일 발송 (라이선스 키)

## 실행

### 1. 의존성 설치

```bash
cd home-next
npm install
```

### 2. 환경 변수 설정

`.env` 파일을 편집합니다:

```env
DATABASE_URL="file:./prisma/dev.db"

# 랜덤 시크릿 생성: openssl rand -base64 32
AUTH_SECRET="여기에_랜덤_시크릿"

# Google Cloud Console → OAuth 2.0 Client ID
# https://console.cloud.google.com/apis/credentials
AUTH_GOOGLE_ID="구글_클라이언트_ID"
AUTH_GOOGLE_SECRET="구글_클라이언트_시크릿"

# (선택) Resend API Key — https://resend.com
RESEND_API_KEY="리센드_API_키"
```

### 3. Google OAuth 설정

1. [Google Cloud Console](https://console.cloud.google.com/apis/credentials)에서 프로젝트 생성
2. OAuth 2.0 Client ID 생성 (웹 애플리케이션)
3. **Authorized redirect URIs** 추가:
   - 개발: `http://localhost:3000/api/auth/callback/google`
   - 프로덕션: `https://your-domain.com/api/auth/callback/google`

### 4. DB 마이그레이션

```bash
npx prisma migrate dev
npx prisma generate
```

### 5. 개발 서버 실행

```bash
npm run dev
```

브라우저에서 `http://localhost:3000` 접속

## 주요 페이지

| 경로 | 설명 |
|------|------|
| `/` | 랜딩 페이지 |
| `/auth/signin` | Google 로그인 |
| `/dashboard` | 라이선스 키 확인, 다운로드 (로그인 필요) |

## API 엔드포인트

| Method | 경로 | 설명 |
|--------|------|------|
| POST | `/api/auth/*` | NextAuth 인증 |
| POST | `/api/serial-key/validate` | 라이선스 키 검증 (Easy Trainer가 호출) |

### 라이선스 키 검증 예시

Easy Trainer 설치 후 라이선스 키를 입력하면, Easy Trainer가 이 API를 호출합니다:

```bash
curl -X POST https://your-domain.com/api/serial-key/validate \
  -H "Content-Type: application/json" \
  -d '{"key": "ET-A1B2C3D4-E5F6", "machineId": "machine-unique-id"}'
```

응답:
```json
{"valid": true, "plan": "free"}
```

## Vercel 배포

```bash
npm i -g vercel
vercel
```

### 커스텀 도메인 연결

1. Vercel Dashboard → Settings → Domains
2. 도메인 입력 (예: `easytrainer.io`)
3. DNS에 Vercel이 안내하는 레코드 추가:
   - `A` 레코드: `76.76.21.21`
   - 또는 `CNAME`: `cname.vercel-dns.com`
4. SSL 인증서 자동 발급 (수 분 소요)

### Vercel 환경 변수 설정

Vercel Dashboard → Settings → Environment Variables에 `.env`의 값들을 등록합니다.

## 파일 구조

```
home-next/
├── prisma/
│   └── schema.prisma              # DB 스키마 (User, SerialKey 등)
├── src/
│   ├── auth.ts                    # NextAuth 설정
│   ├── proxy.ts                   # 라우트 보호 (dashboard)
│   ├── lib/prisma.ts              # Prisma 클라이언트
│   ├── components/
│   │   ├── Navbar.tsx             # 네비게이션
│   │   └── ScrollReveal.tsx       # 스크롤 애니메이션
│   └── app/
│       ├── page.tsx               # 랜딩 페이지
│       ├── auth/signin/page.tsx   # 로그인 페이지
│       ├── dashboard/
│       │   ├── page.tsx           # 대시보드
│       │   └── CopyButtonClient.tsx
│       └── api/
│           ├── auth/[...nextauth]/route.ts
│           └── serial-key/validate/route.ts
├── .env                           # 환경 변수 (git 제외)
└── package.json
```
