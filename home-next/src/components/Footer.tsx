"use client";

import { useState } from "react";
import { COMPANY, type LegalDocKey } from "@/lib/legal";
import LegalModal from "./LegalModal";

export default function Footer() {
  const [openDoc, setOpenDoc] = useState<LegalDocKey | null>(null);

  return (
    <footer className="border-t border-white/5 bg-surface-950">
      <div className="max-w-7xl mx-auto px-6 py-12 space-y-8">
        <div className="flex flex-col md:flex-row md:items-start md:justify-between gap-8">
          <div className="space-y-3">
            <div className="flex items-center gap-2.5">
              <div className="w-7 h-7 rounded-lg bg-gradient-to-br from-indigo-500 to-purple-500 flex items-center justify-center text-[10px] font-extrabold">
                ET
              </div>
              <span className="font-[var(--font-display)] font-bold tracking-tight">
                {COMPANY.name}
              </span>
            </div>
            <dl className="text-xs text-surface-500 leading-relaxed grid grid-cols-[auto_1fr] gap-x-3 gap-y-1 max-w-md">
              <dt className="text-surface-600">상호</dt>
              <dd>{COMPANY.legalName}</dd>
              <dt className="text-surface-600">대표자</dt>
              <dd>{COMPANY.representative}</dd>
              <dt className="text-surface-600">사업자등록번호</dt>
              <dd>{COMPANY.businessNumber}</dd>
              <dt className="text-surface-600">주소</dt>
              <dd>{COMPANY.address}</dd>
              <dt className="text-surface-600">전화</dt>
              <dd>
                <a
                  href={`tel:${COMPANY.phone.replace(/[^0-9+]/g, "")}`}
                  className="hover:text-surface-300"
                >
                  {COMPANY.phone}
                </a>
              </dd>
              <dt className="text-surface-600">이메일</dt>
              <dd>
                <a href={`mailto:${COMPANY.email}`} className="hover:text-surface-300">
                  {COMPANY.email}
                </a>
              </dd>
            </dl>
          </div>

          <div className="flex flex-wrap items-center gap-4 md:flex-col md:items-end md:gap-3">
            <button
              type="button"
              onClick={() => setOpenDoc("terms")}
              className="text-sm text-surface-300 hover:text-white spring-transition cursor-pointer"
            >
              이용약관
            </button>
            <button
              type="button"
              onClick={() => setOpenDoc("privacy")}
              className="text-sm font-semibold text-surface-100 hover:text-white spring-transition cursor-pointer"
            >
              개인정보처리방침
            </button>
            <button
              type="button"
              onClick={() => setOpenDoc("refund")}
              className="text-sm text-surface-300 hover:text-white spring-transition cursor-pointer"
            >
              환불 정책
            </button>
            <a
              href={`mailto:${COMPANY.email}`}
              className="text-sm text-surface-500 hover:text-surface-300 spring-transition"
            >
              문의
            </a>
          </div>
        </div>

        <p className="text-xs text-surface-600 pt-6 border-t border-white/5">
          © 2026 {COMPANY.legalName}. All rights reserved.
        </p>
      </div>

      <LegalModal doc={openDoc} onClose={() => setOpenDoc(null)} />
    </footer>
  );
}
