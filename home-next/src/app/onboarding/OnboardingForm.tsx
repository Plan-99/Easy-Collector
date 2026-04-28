"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import LegalModal from "@/components/LegalModal";
import { LEGAL_TITLES, type LegalDocKey } from "@/lib/legal";

type Initial = {
  organization: string;
  department: string;
  jobRole: string;
};

const AGREEMENTS: { key: LegalDocKey; label: string }[] = [
  { key: "terms", label: "(필수) 이용약관에 동의합니다." },
  { key: "privacy", label: "(필수) 개인정보처리방침에 동의합니다." },
  { key: "refund", label: "(필수) 환불 정책을 확인하고 동의합니다." },
];

export default function OnboardingForm({
  initial,
  nextUrl,
}: {
  initial: Initial;
  nextUrl: string;
}) {
  const router = useRouter();
  const [organization, setOrganization] = useState(initial.organization);
  const [department, setDepartment] = useState(initial.department);
  const [jobRole, setJobRole] = useState(initial.jobRole);
  const [agreed, setAgreed] = useState<Record<LegalDocKey, boolean>>({
    terms: false,
    privacy: false,
    refund: false,
  });
  const [openDoc, setOpenDoc] = useState<LegalDocKey | null>(null);
  const [submitting, setSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const allAgreed = agreed.terms && agreed.privacy && agreed.refund;
  const allFilled =
    organization.trim().length > 0 && department.trim().length > 0 && jobRole.trim().length > 0;
  const canSubmit = allAgreed && allFilled && !submitting;

  function toggleAll(checked: boolean) {
    setAgreed({ terms: checked, privacy: checked, refund: checked });
  }

  async function handleSubmit(e: React.FormEvent) {
    e.preventDefault();
    if (!canSubmit) return;
    setSubmitting(true);
    setError(null);
    try {
      const res = await fetch("/api/onboarding/complete", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          organization: organization.trim(),
          department: department.trim(),
          jobRole: jobRole.trim(),
          agreedTerms: agreed.terms,
          agreedPrivacy: agreed.privacy,
          agreedRefund: agreed.refund,
        }),
      });
      if (!res.ok) {
        const body = await res.json().catch(() => ({}));
        throw new Error(body?.error || "가입 완료에 실패했습니다.");
      }
      router.replace(nextUrl);
      router.refresh();
    } catch (err) {
      setError(err instanceof Error ? err.message : "알 수 없는 오류");
      setSubmitting(false);
    }
  }

  return (
    <>
      <form onSubmit={handleSubmit} className="bezel-card">
        <div className="bezel-inner p-8 space-y-6">
          <Field
            label="소속"
            placeholder="예) (주)이지트레이너 / 한국대학교"
            value={organization}
            onChange={setOrganization}
            required
          />
          <Field
            label="부서"
            placeholder="예) 로보틱스 연구팀"
            value={department}
            onChange={setDepartment}
            required
          />
          <Field
            label="역할"
            placeholder="예) 연구원, PM, 학생"
            value={jobRole}
            onChange={setJobRole}
            required
          />

          <div className="pt-2 border-t border-white/5 space-y-3">
            <label className="flex items-center gap-3 text-sm cursor-pointer select-none">
              <input
                type="checkbox"
                className="w-4 h-4 accent-indigo-500"
                checked={allAgreed}
                onChange={e => toggleAll(e.target.checked)}
              />
              <span className="font-semibold text-surface-100">전체 동의</span>
            </label>

            <div className="pl-7 space-y-2">
              {AGREEMENTS.map(({ key, label }) => (
                <div key={key} className="flex items-center gap-2 text-sm">
                  <label className="flex items-center gap-3 cursor-pointer select-none flex-1 min-w-0">
                    <input
                      type="checkbox"
                      className="w-4 h-4 accent-indigo-500"
                      checked={agreed[key]}
                      onChange={e =>
                        setAgreed(prev => ({ ...prev, [key]: e.target.checked }))
                      }
                    />
                    <span className="text-surface-300 truncate">{label}</span>
                  </label>
                  <button
                    type="button"
                    onClick={() => setOpenDoc(key)}
                    className="text-xs text-indigo-300 hover:text-indigo-200 underline whitespace-nowrap"
                    aria-label={`${LEGAL_TITLES[key]} 보기`}
                  >
                    보기
                  </button>
                </div>
              ))}
            </div>
          </div>

          {error && (
            <p className="text-sm text-red-400 bg-red-500/10 border border-red-500/20 rounded-lg px-3 py-2">
              {error}
            </p>
          )}

          <button
            type="submit"
            disabled={!canSubmit}
            className="w-full py-3.5 rounded-full bg-gradient-to-r from-indigo-500 to-purple-500 text-white font-semibold text-sm shadow-lg shadow-indigo-500/25 hover:shadow-indigo-500/40 spring-transition disabled:opacity-40 disabled:cursor-not-allowed disabled:shadow-none cursor-pointer"
          >
            {submitting ? "처리 중…" : "가입 완료"}
          </button>
        </div>
      </form>

      <LegalModal doc={openDoc} onClose={() => setOpenDoc(null)} />
    </>
  );
}

function Field({
  label,
  value,
  onChange,
  placeholder,
  required,
}: {
  label: string;
  value: string;
  onChange: (v: string) => void;
  placeholder?: string;
  required?: boolean;
}) {
  return (
    <label className="block">
      <span className="text-sm font-semibold text-surface-200">
        {label}
        {required && <span className="text-red-400 ml-0.5">*</span>}
      </span>
      <input
        type="text"
        value={value}
        onChange={e => onChange(e.target.value)}
        placeholder={placeholder}
        required={required}
        className="mt-2 w-full px-4 py-3 rounded-lg bg-surface-900/60 border border-white/10 text-surface-100 text-sm placeholder:text-surface-600 focus:outline-none focus:border-indigo-500/60 focus:ring-2 focus:ring-indigo-500/20 spring-transition"
      />
    </label>
  );
}
