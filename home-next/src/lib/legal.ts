// Single source of truth for company info shown in footer & legal pages.
// PortOne requires: 상호 / 대표자 / 사업자등록번호 / 주소 / 유선번호 / 이메일.
// Replace each TODO with real value before signing the PortOne merchant contract.
export const COMPANY = {
  name: "Vertical Labs Inc.",
  legalName: "버티컬랩스 주식회사",
  representative: "남창주",
  businessNumber: "558-88-03692", // 사업자등록번호
  ecommerceNumber: "", // 통신판매업신고번호
  address: "서울특별시 마포구 백범로 35 TE 415호",
  phone: "02-964-0730",
  email: "hcjung@vertic-ai.com",
  websiteUrl: "https://easytrainerhome.vercel.app",
} as const;

// Used in dashboard "최근 업데이트" copy + legal modals.
export const LEGAL_LAST_UPDATED = "2026-04-28";

export type LegalDocKey = "terms" | "privacy" | "refund";

export const LEGAL_TITLES: Record<LegalDocKey, string> = {
  terms: "이용약관",
  privacy: "개인정보처리방침",
  refund: "환불 정책",
};

// Section = { heading, paragraphs }. Rendered with <h3> + <p>.
type Section = { heading: string; paragraphs: string[] };

export const TERMS_SECTIONS: Section[] = [
  {
    heading: "제1조 (목적)",
    paragraphs: [
      `본 약관은 ${COMPANY.legalName}(이하 "회사")가 제공하는 Easy Trainer 서비스(이하 "서비스")의 이용과 관련하여 회사와 회원 간의 권리·의무 및 책임사항을 정함을 목적으로 합니다.`,
    ],
  },
  {
    heading: "제2조 (정의)",
    paragraphs: [
      `1. "회원"이라 함은 본 약관에 동의하고 회사가 제공하는 서비스를 이용하는 자를 말합니다.`,
      `2. "콘텐츠"라 함은 회사가 제공하는 모듈, 데이터셋, 학습 기능 등 일체의 디지털 자료를 말합니다.`,
      `3. "유료 모듈"이라 함은 별도의 결제를 통해 영구 사용 권한을 부여받는 모듈을 말합니다.`,
    ],
  },
  {
    heading: "제3조 (약관의 효력 및 변경)",
    paragraphs: [
      `1. 본 약관은 회원이 회원가입 시 동의함으로써 효력이 발생합니다.`,
      `2. 회사는 관련 법령을 위배하지 않는 범위 내에서 본 약관을 개정할 수 있으며, 변경사항은 시행일 7일 전(불리한 변경의 경우 30일 전)부터 서비스 내 공지를 통해 안내합니다.`,
    ],
  },
  {
    heading: "제4조 (회원가입)",
    paragraphs: [
      `1. 회원가입은 Google 계정 인증을 통해 이루어지며, 회원이 본 약관 및 개인정보처리방침에 동의함으로써 가입이 완료됩니다.`,
      `2. 회사는 다음 각 호에 해당하는 신청에 대해 가입을 거부할 수 있습니다.`,
      `   - 타인의 명의로 신청한 경우`,
      `   - 회원가입 신청 시 허위 정보를 기재한 경우`,
      `   - 사회 질서 또는 미풍양속을 저해할 목적으로 신청한 경우`,
    ],
  },
  {
    heading: "제5조 (서비스의 제공)",
    paragraphs: [
      `1. 회사는 회원이 Easy Trainer 런처를 통해 모듈 마켓, 학습, 데이터 수집 등의 기능을 이용할 수 있도록 합니다.`,
      `2. 회사는 시스템 점검, 천재지변 등 불가항력적 사유 발생 시 서비스 제공을 일시적으로 중단할 수 있으며, 이로 인한 손해에 대하여 회사는 회원에게 사전 고지된 범위 내에서 책임을 부담합니다.`,
    ],
  },
  {
    heading: "제6조 (회원의 의무)",
    paragraphs: [
      `회원은 본 서비스를 이용함에 있어 다음 각 호의 행위를 하여서는 아니 됩니다.`,
      `- 타인의 계정을 도용하거나 부정 사용하는 행위`,
      `- 회사의 사전 승낙 없이 서비스를 영리 목적으로 사용하는 행위`,
      `- 서비스의 안정적 운영을 방해하는 일체의 행위`,
      `- 모듈 또는 콘텐츠를 무단으로 복제·배포·재판매하는 행위`,
    ],
  },
  {
    heading: "제7조 (계약 해지 및 이용 제한)",
    paragraphs: [
      `1. 회원은 언제든지 마이페이지에서 회원 탈퇴를 요청할 수 있으며, 회사는 즉시 처리합니다.`,
      `2. 회사는 회원이 본 약관을 위반한 경우 사전 통지 후 서비스 이용을 제한하거나 회원 자격을 박탈할 수 있습니다.`,
    ],
  },
  {
    heading: "제8조 (책임의 제한)",
    paragraphs: [
      `회사는 천재지변, 회원의 귀책사유, 제3자 서비스 장애 등 회사의 합리적 통제를 벗어난 사유로 발생한 손해에 대해서는 책임을 지지 않습니다.`,
    ],
  },
  {
    heading: "제9조 (준거법 및 관할)",
    paragraphs: [
      `본 약관은 대한민국 법령을 준거법으로 하며, 본 서비스 이용으로 발생한 분쟁에 대한 소송의 관할 법원은 회사의 본점 소재지를 관할하는 법원으로 합니다.`,
    ],
  },
  {
    heading: "부칙",
    paragraphs: [`본 약관은 ${LEGAL_LAST_UPDATED}부터 시행됩니다.`],
  },
];

export const PRIVACY_SECTIONS: Section[] = [
  {
    heading: "1. 수집하는 개인정보 항목",
    paragraphs: [
      `회사는 다음과 같은 개인정보를 수집합니다.`,
      `- 필수: 이메일, 이름, 프로필 이미지(Google 계정 제공)`,
      `- 필수: 소속, 부서, 역할`,
      `- 결제 시: 결제 수단 정보(카드 별칭·말번 4자리만 저장, 원본 카드번호는 PortOne(주) 빌링키로 대체)`,
      `- 자동 수집: 머신 식별자, 호스트명, OS, 접속 IP, 접속 시각`,
    ],
  },
  {
    heading: "2. 개인정보의 수집·이용 목적",
    paragraphs: [
      `- 회원 식별 및 본인 확인`,
      `- 서비스 제공(모듈 다운로드, 학습 자원 할당)`,
      `- 결제 처리 및 영수증 발급`,
      `- 부정 이용 방지 및 보안`,
      `- 고객 문의 응대`,
    ],
  },
  {
    heading: "3. 개인정보의 보유 및 이용 기간",
    paragraphs: [
      `회원 탈퇴 시 즉시 파기함을 원칙으로 하나, 다음 정보는 관련 법령에 따라 일정 기간 보관합니다.`,
      `- 결제 기록: 5년 (전자상거래법)`,
      `- 접속 로그: 3개월 (통신비밀보호법)`,
    ],
  },
  {
    heading: "4. 개인정보의 제3자 제공",
    paragraphs: [
      `회사는 다음의 경우에 한해 개인정보를 제3자에게 제공합니다.`,
      `- 결제: 결제 수단 정보 → PortOne(주) 및 연계된 PG사(토스페이먼츠, 카카오페이 등)`,
      `- 법령에 따른 요청 시 수사기관`,
      `상기 외 회원의 사전 동의 없이는 제공되지 않습니다.`,
    ],
  },
  {
    heading: "5. 개인정보 처리 위탁",
    paragraphs: [
      `- 클라우드 호스팅: Vercel Inc., Neon, Inc.`,
      `- 결제 처리: PortOne(주)`,
      `- 인증: Google LLC`,
    ],
  },
  {
    heading: "6. 정보주체의 권리",
    paragraphs: [
      `회원은 언제든지 본인의 개인정보를 조회·정정·삭제·처리정지 요청할 수 있으며, 마이페이지 또는 ${COMPANY.email}로 연락하여 행사할 수 있습니다.`,
    ],
  },
  {
    heading: "7. 개인정보 보호책임자",
    paragraphs: [
      `- 책임자: ${COMPANY.representative}`,
      `- 연락처: ${COMPANY.phone}`,
      `- 이메일: ${COMPANY.email}`,
    ],
  },
  {
    heading: "부칙",
    paragraphs: [`본 방침은 ${LEGAL_LAST_UPDATED}부터 시행됩니다.`],
  },
];

export const REFUND_SECTIONS: Section[] = [
  {
    heading: "1. 적용 범위",
    paragraphs: [
      `본 환불 정책은 Easy Trainer에서 판매되는 유료 모듈의 청약 철회 및 환불에 관한 사항을 규정합니다.`,
    ],
  },
  {
    heading: "2. 청약 철회 (전자상거래법 제17조)",
    paragraphs: [
      `회원은 결제일로부터 7일 이내에 청약을 철회하고 전액 환불을 요청할 수 있습니다. 단, 다음의 경우 청약 철회가 제한됩니다.`,
      `- 모듈을 다운로드하여 설치한 경우`,
      `- 모듈에 포함된 디지털 콘텐츠를 사용한 경우`,
      `(전자상거래 등에서의 소비자보호에 관한 법률 시행령 제21조에 따른 디지털 콘텐츠 예외 적용)`,
    ],
  },
  {
    heading: "3. 부분 환불 / 사용 후 환불",
    paragraphs: [
      `다운로드 후 환불은 원칙적으로 불가하나, 다음의 경우 회사 검토 후 환불이 가능합니다.`,
      `- 모듈에 명시된 기능이 작동하지 않는 경우`,
      `- 회사 측 시스템 장애로 정상 사용이 7일 이상 불가한 경우`,
    ],
  },
  {
    heading: "4. 환불 절차",
    paragraphs: [
      `1. 마이페이지 → 결제 내역 → 환불 요청`,
      `2. 또는 ${COMPANY.email}로 결제 정보(주문번호, 결제일)와 함께 사유 전달`,
      `3. 회사는 영업일 기준 3일 이내 환불 가능 여부를 회신합니다.`,
      `4. 환불 승인 시 결제 수단으로 영업일 기준 3~5일 내 환급됩니다(카드사 정책에 따라 1주 이상 소요될 수 있음).`,
    ],
  },
  {
    heading: "5. 환불 수수료",
    paragraphs: [
      `청약 철회 기간 내(결제 7일 이내, 미사용)의 환불은 수수료 없이 전액 환불됩니다. 회사 귀책 사유 외에는 PG사 환불 수수료가 차감될 수 있습니다.`,
    ],
  },
  {
    heading: "부칙",
    paragraphs: [`본 정책은 ${LEGAL_LAST_UPDATED}부터 시행됩니다.`],
  },
];

export const LEGAL_SECTIONS: Record<LegalDocKey, Section[]> = {
  terms: TERMS_SECTIONS,
  privacy: PRIVACY_SECTIONS,
  refund: REFUND_SECTIONS,
};
