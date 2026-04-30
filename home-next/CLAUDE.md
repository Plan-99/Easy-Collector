# Easy Trainer Homepage

## Overview
Easy Trainer의 공식 웹사이트 + 백오피스. 제품 소개, 설치, **모듈 마켓 결제**, 사용자/기기/구매 내역 관리.

## 인증 (시리얼 키 → Google OAuth 디바이스 플로우)
시리얼 키 방식은 폐기됨. 런처가 home-next의 OAuth-디바이스-플로우로 인증한다.

```
런처 → POST /api/device-auth/start { machineId, hostname, os }
                                    → { nonce, verificationUrl, expiresAt }
런처 → 기본 브라우저 열기 → /auth/device?code=<nonce>
사용자 → Google 로그인 → "이 기기 등록" 클릭
브라우저 → POST /api/device-auth/approve { nonce, action: "approve" }
                                    → Device row + accessToken 발급
런처 → GET /api/device-auth/poll?nonce=...  (2초 폴링)
                                    → { status: "APPROVED", accessToken, user, device }
런처 → ~/.config/easytrainer/auth.json 에 저장
이후 모든 호출: Authorization: Bearer <accessToken>
```

룰: **사용자당 활성 Device 1개**. 다른 머신에서 새로 시도하면 `DEVICE_ALREADY_BOUND`로 차단.
사용자는 `/dashboard`에서 직접 unbind 가능.

## 모듈 카탈로그 + 결제

### 데이터 모델 (Prisma)
- `Module` — 카탈로그 (id, name, category, priceKrw, active). 가격은 admin이 DB에서 직접 설정.
- `Entitlement` — userId × moduleId 영구 보유 (1회 결제 후 보유). `priceKrw=0` 모듈은 row 없이 묵시적으로 보유.
- `Payment` — PortOne 결제 내역 (PENDING/PAID/FAILED/CANCELLED/REFUNDED).
- `BillingKey` — PortOne 빌링키 (저장된 카드). 원시 PAN은 절대 저장하지 않음.
- `Device` / `DeviceAuthRequest` — 인증/세션.

### 결제 (PortOne v2)
- 채널: 갤럭시아머니트리 (테스트 channelKey 가맹점 단위 발급). 카드/카카오페이 SDK 단일 진입점.
- 런처에서 [결제 ₩XX,XXX] 클릭 → 브라우저로 `/checkout/{moduleId}` 열기 → PortOne Browser SDK 호출 → PG 결제창 → 완료 시 `/api/payments/complete`로 서버 검증 → `Entitlement` 생성.
- 서버측 검증: `applyPaidPayment` (`src/lib/payments.ts`)이 PortOne REST `getPayment` 결과로 status=PAID + 금액 일치를 검증한 뒤 트랜잭션으로 Payment 업데이트 + Entitlement upsert. `/complete`와 `/webhook` 둘 다 동일 함수를 호출 → 멱등.
- 환불: `/api/payments/[id]/refund` (사용자 7일 윈도우, `paidAt` 기준), `/api/admin/payments/[id]/refund` (관리자, 윈도우 무시). 모두 PortOne `cancelPayment` 호출 후 `applyNonPaidPayment`로 entitlement 회수.
- 빌링키: 미구현. `BillingKey` 모델만 있음. 다음 단계 작업.

### 필요 환경변수 (Vercel에 추가)
- `PORTONE_STORE_ID` — `store-fe1ec194-20b7-4e61-b9e2-fd35dff38561` (테스트)
- `PORTONE_CHANNEL_KEY` — 갤럭시아 채널 키 (포트원 콘솔 → 결제 연동 → 채널)
- `PORTONE_V2_API_SECRET` — 서버 검증용 (포트원 개발자센터 → API Keys)
- `PORTONE_WEBHOOK_SECRET` — 웹훅 시그니처 검증용

## API 엔드포인트
- 공개: `GET /api/modules` (카탈로그+가격)
- 디바이스 플로우: `POST /api/device-auth/start`, `GET /api/device-auth/poll`, `POST /api/device-auth/approve`
- Bearer 인증 (런처): `GET /api/me`, `GET /api/entitlements`
- 세션 인증 (대시보드): `POST /api/devices/[id]/unbind`, `POST /api/admin/devices/[id]/unbind`
- 결제: `POST /api/payments/checkout`, `POST /api/payments/complete`, `POST /api/payments/[id]/refund`, `POST /api/payments/webhook`
- 관리자: `POST /api/admin/entitlements` (부여), `DELETE /api/admin/entitlements?userId=&moduleId=` (회수), `POST /api/admin/payments/[id]/refund`

## 페이지
- `/` — 랜딩 + 가격 플랜
- `/auth/signin` — Google OAuth (성공 시 `/onboarding`으로)
- `/onboarding` — 첫 로그인 후 소속/부서/역할 입력 + 이용약관/개인정보/환불 동의. `/dashboard`·`/admin`·`/auth/device`는 미완료 시 여기로 강제 redirect
- `/auth/device?code=` — 런처 디바이스 등록 동의
- `/checkout/[moduleId]` — 단일 모듈 결제 페이지 (PortOne Browser SDK)
- `/dashboard` — 내 정보: 연결 기기 / 보유 모듈 / 결제 내역 (7일 이내 환불 버튼) / 저장 카드
- `/admin` — 사용자/기기/결제 관리

## 약관/회사 정보
- 푸터 컴포넌트(`src/components/Footer.tsx`)가 사업자 정보(상호/대표자/사업자등록번호/통신판매업/주소/유선번호/이메일)를 표시하고 이용약관·개인정보처리방침·환불 정책 모달을 트리거함.
- 회사 실값/약관 본문은 `src/lib/legal.ts`에 모여 있음. PortOne 계약 직전 `COMPANY.representative`, `businessNumber`, `ecommerceNumber`, `address`, `phone` 의 `TODO_*` 마커를 실제 값으로 교체.

## 기술 스택
- Next.js 16 (App Router, Turbopack), React 19
- NextAuth v5 (Google provider) + PrismaAdapter
- Prisma 7 + Neon Postgres (`@prisma/adapter-pg`)
- Tailwind v4
- `@portone/browser-sdk` ^0.1.5, `@portone/server-sdk` ^0.19.0

## 참고
- 이 폴더는 Easy-Collector 본체와 별개의 웹사이트 프로젝트
- 런처(`release/ui/`)와는 HTTP API로만 통신 (`license_server_url` config 키)
- DB 마이그레이션은 비대화형 환경이라 `prisma migrate diff --from-config-datasource --to-schema --script -o ...` 후 `prisma migrate deploy`로 적용
