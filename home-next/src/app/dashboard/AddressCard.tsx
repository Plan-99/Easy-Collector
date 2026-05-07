"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { openPostcode } from "@/lib/postcode";
import { formatPhoneKr } from "@/lib/phone";

type Address = {
  id: string;
  label: string | null;
  recipientName: string;
  phone: string;
  postalCode: string;
  addressLine1: string;
  addressLine2: string | null;
  requestNote: string | null;
  isDefault: boolean;
};

type Form = {
  label: string;
  recipientName: string;
  phone: string;
  postalCode: string;
  addressLine1: string;
  addressLine2: string;
  requestNote: string;
  isDefault: boolean;
};

const empty: Form = {
  label: "",
  recipientName: "",
  phone: "",
  postalCode: "",
  addressLine1: "",
  addressLine2: "",
  requestNote: "",
  isDefault: false,
};

export default function AddressCard({ initial }: { initial: Address[] }) {
  const router = useRouter();
  const [editing, setEditing] = useState<string | "new" | null>(null);
  const [form, setForm] = useState<Form>(empty);
  const [submitting, setSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);

  function startNew() {
    setEditing("new");
    setForm({ ...empty, isDefault: initial.length === 0 });
    setError(null);
  }
  function startEdit(a: Address) {
    setEditing(a.id);
    setForm({
      label: a.label || "",
      recipientName: a.recipientName,
      phone: formatPhoneKr(a.phone),
      postalCode: a.postalCode,
      addressLine1: a.addressLine1,
      addressLine2: a.addressLine2 || "",
      requestNote: a.requestNote || "",
      isDefault: a.isDefault,
    });
    setError(null);
  }
  function cancel() {
    setEditing(null);
    setError(null);
  }

  async function save() {
    setSubmitting(true);
    setError(null);
    try {
      const body = {
        label: form.label.trim() || null,
        recipientName: form.recipientName.trim(),
        phone: form.phone.trim(),
        postalCode: form.postalCode.trim(),
        addressLine1: form.addressLine1.trim(),
        addressLine2: form.addressLine2.trim() || null,
        requestNote: form.requestNote.trim() || null,
        isDefault: form.isDefault,
      };
      const url =
        editing === "new" ? "/api/account/addresses" : `/api/account/addresses/${editing}`;
      const method = editing === "new" ? "POST" : "PATCH";
      const res = await fetch(url, {
        method,
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      if (!res.ok) {
        const b = await res.json().catch(() => ({}));
        throw new Error(b?.error || "저장 실패");
      }
      cancel();
      router.refresh();
    } catch (err) {
      setError(err instanceof Error ? err.message : "알 수 없는 오류");
    } finally {
      setSubmitting(false);
    }
  }

  async function remove(id: string) {
    if (!confirm("이 배송지를 삭제하시겠습니까?")) return;
    setSubmitting(true);
    try {
      const res = await fetch(`/api/account/addresses/${id}`, { method: "DELETE" });
      if (!res.ok) throw new Error("삭제 실패");
      router.refresh();
    } catch (err) {
      alert(err instanceof Error ? err.message : "알 수 없는 오류");
    } finally {
      setSubmitting(false);
    }
  }

  return (
    <div className="bezel-card mb-8">
      <div className="bezel-inner p-8">
        <div className="flex items-center justify-between mb-1">
          <h3 className="font-[var(--font-display)] font-bold text-lg">배송지</h3>
          {editing === null && (
            <button
              type="button"
              onClick={startNew}
              className="text-xs px-3 py-1.5 rounded-full bg-indigo-500/20 hover:bg-indigo-500/30 border border-indigo-500/30 text-indigo-200 font-semibold"
            >
              + 배송지 추가
            </button>
          )}
        </div>
        <p className="text-surface-500 text-sm mb-6">
          저장된 배송지는 하드웨어 주문 시 자동으로 채워집니다.
        </p>

        {editing !== null && (
          <div className="mb-4 p-4 rounded-xl bg-surface-900/60 border border-white/10 space-y-3">
            <div className="grid grid-cols-2 gap-3">
              <Field label="별칭">
                <input
                  type="text"
                  value={form.label}
                  onChange={e => setForm(f => ({ ...f, label: e.target.value }))}
                  placeholder="예: 집, 회사"
                  className={inputCls}
                />
              </Field>
              <Field label="받는 사람" required>
                <input
                  type="text"
                  value={form.recipientName}
                  onChange={e => setForm(f => ({ ...f, recipientName: e.target.value }))}
                  className={inputCls}
                />
              </Field>
            </div>
            <div className="grid grid-cols-2 gap-3">
              <Field label="연락처" required>
                <input
                  type="tel"
                  value={form.phone}
                  onChange={e => setForm(f => ({ ...f, phone: formatPhoneKr(e.target.value) }))}
                  placeholder="010-0000-0000"
                  inputMode="numeric"
                  maxLength={13}
                  className={inputCls}
                />
              </Field>
              <Field label="우편번호" required>
                <div className="flex gap-2">
                  <input
                    type="text"
                    value={form.postalCode}
                    onChange={e => setForm(f => ({ ...f, postalCode: e.target.value }))}
                    placeholder="00000"
                    maxLength={6}
                    className={inputCls + " font-mono"}
                    readOnly
                  />
                  <button
                    type="button"
                    onClick={async () => {
                      const r = await openPostcode();
                      if (!r) return;
                      setForm(f => ({
                        ...f,
                        postalCode: r.zonecode,
                        addressLine1: r.roadAddress,
                      }));
                    }}
                    className="shrink-0 px-3 py-2 rounded-lg bg-indigo-500/20 hover:bg-indigo-500/30 border border-indigo-500/30 text-indigo-200 text-xs font-semibold whitespace-nowrap"
                  >
                    주소 검색
                  </button>
                </div>
              </Field>
            </div>
            <Field label="주소" required>
              <input
                type="text"
                value={form.addressLine1}
                onChange={e => setForm(f => ({ ...f, addressLine1: e.target.value }))}
                placeholder="주소 검색을 눌러주세요"
                className={inputCls}
                readOnly
              />
            </Field>
            <Field label="상세 주소">
              <input
                type="text"
                value={form.addressLine2}
                onChange={e => setForm(f => ({ ...f, addressLine2: e.target.value }))}
                placeholder="동·호수·기타"
                className={inputCls}
              />
            </Field>
            <Field label="배송 요청 사항">
              <input
                type="text"
                value={form.requestNote}
                onChange={e => setForm(f => ({ ...f, requestNote: e.target.value }))}
                placeholder="문 앞에 두세요 등"
                className={inputCls}
              />
            </Field>
            <label className="flex items-center gap-2 text-sm cursor-pointer">
              <input
                type="checkbox"
                className="w-4 h-4 accent-indigo-500"
                checked={form.isDefault}
                onChange={e => setForm(f => ({ ...f, isDefault: e.target.checked }))}
              />
              <span className="text-surface-300">기본 배송지로 설정</span>
            </label>
            {error && (
              <p className="text-xs text-red-400 bg-red-500/10 border border-red-500/20 rounded-md px-3 py-2">
                {error}
              </p>
            )}
            <div className="flex gap-2 justify-end pt-2">
              <button
                type="button"
                onClick={cancel}
                className="px-4 py-2 rounded-full bg-white/5 border border-white/10 text-xs"
              >
                취소
              </button>
              <button
                type="button"
                onClick={save}
                disabled={submitting}
                className="px-5 py-2 rounded-full bg-gradient-to-r from-indigo-500 to-purple-500 text-white text-xs font-semibold disabled:opacity-40"
              >
                {submitting ? "저장 중…" : "저장"}
              </button>
            </div>
          </div>
        )}

        {initial.length === 0 ? (
          editing === null && (
            <p className="text-surface-500 text-sm">
              저장된 배송지가 없습니다. 첫 주문 시 입력한 배송지를 저장할 수 있습니다.
            </p>
          )
        ) : (
          <ul className="space-y-2">
            {initial.map(a => (
              <li
                key={a.id}
                className="rounded-xl bg-surface-900/60 border border-white/5 p-4"
              >
                <div className="flex items-start justify-between gap-3">
                  <div className="min-w-0 flex-1 text-sm">
                    <div className="flex items-center gap-2">
                      {a.label && (
                        <span className="text-xs px-2 py-0.5 rounded bg-white/10 text-surface-200 font-semibold">
                          {a.label}
                        </span>
                      )}
                      {a.isDefault && (
                        <span className="text-[10px] px-1.5 py-0.5 rounded bg-indigo-500/20 text-indigo-300 font-semibold">
                          기본
                        </span>
                      )}
                      <span className="font-medium text-surface-100">{a.recipientName}</span>
                      <span className="text-surface-500 text-xs">{a.phone}</span>
                    </div>
                    <p className="text-surface-300 mt-1">
                      ({a.postalCode}) {a.addressLine1}
                      {a.addressLine2 ? ` ${a.addressLine2}` : ""}
                    </p>
                    {a.requestNote && (
                      <p className="text-surface-500 text-xs mt-1">{a.requestNote}</p>
                    )}
                  </div>
                  <div className="flex gap-2 shrink-0">
                    <button
                      type="button"
                      onClick={() => startEdit(a)}
                      className="text-xs text-indigo-300 hover:text-indigo-200 underline"
                    >
                      편집
                    </button>
                    <button
                      type="button"
                      onClick={() => remove(a.id)}
                      disabled={submitting}
                      className="text-xs text-red-300 hover:text-red-200 underline disabled:opacity-50"
                    >
                      삭제
                    </button>
                  </div>
                </div>
              </li>
            ))}
          </ul>
        )}
      </div>
    </div>
  );
}

const inputCls =
  "w-full px-3 py-2 rounded-lg bg-surface-900/60 border border-white/10 text-surface-100 text-sm placeholder:text-surface-600 focus:outline-none focus:border-indigo-500/60 focus:ring-2 focus:ring-indigo-500/20";

function Field({
  label,
  required,
  children,
}: {
  label: string;
  required?: boolean;
  children: React.ReactNode;
}) {
  return (
    <label className="block">
      <span className="text-xs font-semibold text-surface-200">
        {label}
        {required && <span className="text-red-400 ml-0.5">*</span>}
      </span>
      <div className="mt-1.5">{children}</div>
    </label>
  );
}
