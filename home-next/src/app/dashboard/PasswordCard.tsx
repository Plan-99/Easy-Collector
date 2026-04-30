"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";

export default function PasswordCard({ hasPassword }: { hasPassword: boolean }) {
  const router = useRouter();
  const [open, setOpen] = useState(false);
  const [currentPassword, setCurrentPassword] = useState("");
  const [newPassword, setNewPassword] = useState("");
  const [confirmPassword, setConfirmPassword] = useState("");
  const [submitting, setSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);

  const tooShort = newPassword.length > 0 && newPassword.length < 8;
  const mismatch = !!confirmPassword && newPassword !== confirmPassword;
  const canSubmit =
    !submitting &&
    newPassword.length >= 8 &&
    newPassword === confirmPassword &&
    (!hasPassword || currentPassword.length > 0);

  function reset() {
    setCurrentPassword("");
    setNewPassword("");
    setConfirmPassword("");
    setError(null);
  }

  async function handleSubmit(e: React.FormEvent) {
    e.preventDefault();
    if (!canSubmit) return;
    setSubmitting(true);
    setError(null);
    setSuccess(null);
    try {
      const res = await fetch("/api/account/password", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          currentPassword: hasPassword ? currentPassword : undefined,
          newPassword,
        }),
      });
      if (!res.ok) {
        const body = await res.json().catch(() => ({}));
        const msg =
          body?.error === "WRONG_CURRENT_PASSWORD"
            ? "현재 비밀번호가 일치하지 않습니다."
            : body?.error === "PASSWORD_TOO_SHORT"
              ? "비밀번호는 8자 이상이어야 합니다."
              : "비밀번호 변경에 실패했습니다.";
        throw new Error(msg);
      }
      setSuccess(hasPassword ? "비밀번호가 변경되었습니다." : "비밀번호가 설정되었습니다.");
      reset();
      setOpen(false);
      router.refresh();
    } catch (err) {
      setError(err instanceof Error ? err.message : "알 수 없는 오류");
    } finally {
      setSubmitting(false);
    }
  }

  async function handleRemove() {
    if (!confirm("비밀번호 로그인을 해제하시겠습니까? 이후엔 Google로만 로그인할 수 있습니다.")) return;
    setSubmitting(true);
    try {
      const res = await fetch("/api/account/password", { method: "DELETE" });
      if (!res.ok) throw new Error("해제 실패");
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
        <h3 className="font-[var(--font-display)] font-bold text-lg mb-1">로그인 방식</h3>
        <p className="text-surface-500 text-sm mb-6">
          Google 로그인 외에 이메일·비밀번호로도 로그인할 수 있도록 설정합니다.
        </p>

        <div className="flex items-center justify-between gap-4">
          <div className="text-sm">
            <p className="font-medium text-surface-100">
              비밀번호 로그인:{" "}
              <span className={hasPassword ? "text-emerald-400" : "text-surface-400"}>
                {hasPassword ? "사용 중" : "미설정"}
              </span>
            </p>
            <p className="text-surface-500 text-xs mt-1">
              {hasPassword
                ? "이메일과 비밀번호로 언제든지 로그인할 수 있습니다."
                : "비밀번호를 설정하면 이메일과 비밀번호 조합으로 로그인할 수 있습니다."}
            </p>
          </div>
          <div className="flex flex-col items-end gap-2 shrink-0">
            <button
              type="button"
              onClick={() => {
                reset();
                setSuccess(null);
                setOpen(o => !o);
              }}
              className="px-4 py-2 rounded-full bg-indigo-500/20 hover:bg-indigo-500/30 border border-indigo-500/30 text-indigo-100 text-xs font-semibold spring-transition cursor-pointer"
            >
              {open ? "닫기" : hasPassword ? "비밀번호 변경" : "비밀번호 설정"}
            </button>
            {hasPassword && !open && (
              <button
                type="button"
                onClick={handleRemove}
                disabled={submitting}
                className="text-xs text-red-300 hover:text-red-200 underline disabled:opacity-50 cursor-pointer"
              >
                비밀번호 로그인 해제
              </button>
            )}
          </div>
        </div>

        {success && !open && (
          <p className="mt-4 text-xs text-emerald-300 bg-emerald-500/10 border border-emerald-500/20 rounded-md px-3 py-2">
            {success}
          </p>
        )}

        {open && (
          <form onSubmit={handleSubmit} className="mt-6 pt-6 border-t border-white/5 space-y-3">
            {hasPassword && (
              <input
                type="password"
                placeholder="현재 비밀번호"
                value={currentPassword}
                onChange={e => setCurrentPassword(e.target.value)}
                autoComplete="current-password"
                className="w-full px-4 py-2.5 rounded-lg bg-surface-900/60 border border-white/10 text-surface-100 text-sm placeholder:text-surface-600 focus:outline-none focus:border-indigo-500/60 focus:ring-2 focus:ring-indigo-500/20"
              />
            )}
            <input
              type="password"
              placeholder="새 비밀번호 (8자 이상)"
              value={newPassword}
              onChange={e => setNewPassword(e.target.value)}
              autoComplete="new-password"
              className="w-full px-4 py-2.5 rounded-lg bg-surface-900/60 border border-white/10 text-surface-100 text-sm placeholder:text-surface-600 focus:outline-none focus:border-indigo-500/60 focus:ring-2 focus:ring-indigo-500/20"
            />
            <input
              type="password"
              placeholder="새 비밀번호 확인"
              value={confirmPassword}
              onChange={e => setConfirmPassword(e.target.value)}
              autoComplete="new-password"
              className="w-full px-4 py-2.5 rounded-lg bg-surface-900/60 border border-white/10 text-surface-100 text-sm placeholder:text-surface-600 focus:outline-none focus:border-indigo-500/60 focus:ring-2 focus:ring-indigo-500/20"
            />
            {tooShort && <p className="text-xs text-amber-300">비밀번호는 8자 이상이어야 합니다.</p>}
            {mismatch && <p className="text-xs text-amber-300">비밀번호가 일치하지 않습니다.</p>}
            {error && (
              <p className="text-xs text-red-400 bg-red-500/10 border border-red-500/20 rounded-md px-3 py-2">
                {error}
              </p>
            )}
            <button
              type="submit"
              disabled={!canSubmit}
              className="w-full py-2.5 rounded-full bg-gradient-to-r from-indigo-500 to-purple-500 text-white text-sm font-semibold spring-transition disabled:opacity-40 disabled:cursor-not-allowed cursor-pointer"
            >
              {submitting ? "저장 중…" : hasPassword ? "변경" : "설정"}
            </button>
          </form>
        )}
      </div>
    </div>
  );
}
