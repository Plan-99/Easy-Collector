"use client";

import { useEffect } from "react";
import { LEGAL_LAST_UPDATED, LEGAL_SECTIONS, LEGAL_TITLES, type LegalDocKey } from "@/lib/legal";

export default function LegalModal({
  doc,
  onClose,
}: {
  doc: LegalDocKey | null;
  onClose: () => void;
}) {
  useEffect(() => {
    if (!doc) return;
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape") onClose();
    };
    document.addEventListener("keydown", onKey);
    document.body.style.overflow = "hidden";
    return () => {
      document.removeEventListener("keydown", onKey);
      document.body.style.overflow = "";
    };
  }, [doc, onClose]);

  if (!doc) return null;

  const sections = LEGAL_SECTIONS[doc];
  const title = LEGAL_TITLES[doc];

  return (
    <div
      className="fixed inset-0 z-[100] flex items-center justify-center px-4 py-8 bg-black/70 backdrop-blur-sm"
      role="dialog"
      aria-modal="true"
      aria-labelledby="legal-modal-title"
      onClick={onClose}
    >
      <div
        className="relative w-full max-w-2xl max-h-[85vh] flex flex-col rounded-2xl bg-surface-900 border border-white/10 shadow-2xl"
        onClick={e => e.stopPropagation()}
      >
        <div className="flex items-center justify-between px-6 py-5 border-b border-white/5">
          <div>
            <h2 id="legal-modal-title" className="font-[var(--font-display)] font-bold text-lg">
              {title}
            </h2>
            <p className="text-surface-500 text-xs mt-0.5">시행일자 {LEGAL_LAST_UPDATED}</p>
          </div>
          <button
            type="button"
            onClick={onClose}
            aria-label="닫기"
            className="w-9 h-9 flex items-center justify-center rounded-full bg-white/5 hover:bg-white/10 text-surface-300 hover:text-white spring-transition"
          >
            ✕
          </button>
        </div>

        <div className="overflow-y-auto px-6 py-6 space-y-6 text-sm text-surface-300 leading-relaxed">
          {sections.map(s => (
            <section key={s.heading}>
              <h3 className="font-bold text-surface-100 mb-2">{s.heading}</h3>
              <div className="space-y-2 text-[0.92rem]">
                {s.paragraphs.map((p, i) => (
                  <p key={i} className="whitespace-pre-line">
                    {p}
                  </p>
                ))}
              </div>
            </section>
          ))}
        </div>

        <div className="px-6 py-4 border-t border-white/5 flex justify-end">
          <button
            type="button"
            onClick={onClose}
            className="px-5 py-2 rounded-full bg-white/10 hover:bg-white/15 border border-white/10 text-sm font-semibold spring-transition"
          >
            닫기
          </button>
        </div>
      </div>
    </div>
  );
}
