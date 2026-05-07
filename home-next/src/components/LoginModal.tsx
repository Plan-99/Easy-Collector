"use client";

import { useEffect, useState } from "react";
import { signIn } from "next-auth/react";

export default function LoginModal({
  open,
  onClose,
  callbackUrl,
  title = "로그인 후 결제 가능합니다",
  description = "Google 또는 이메일·비밀번호로 로그인 후 결제를 진행할 수 있습니다.",
}: {
  open: boolean;
  onClose: () => void;
  callbackUrl: string;
  title?: string;
  description?: string;
}) {
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [submitting, setSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (!open) return;
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape") onClose();
    };
    document.addEventListener("keydown", onKey);
    document.body.style.overflow = "hidden";
    return () => {
      document.removeEventListener("keydown", onKey);
      document.body.style.overflow = "";
    };
  }, [open, onClose]);

  if (!open) return null;

  async function handleCredentials(e: React.FormEvent) {
    e.preventDefault();
    if (!email.trim() || !password) return;
    setSubmitting(true);
    setError(null);
    const res = await signIn("credentials", {
      email: email.trim(),
      password,
      redirect: false,
    });
    if (res?.error) {
      setSubmitting(false);
      setError("이메일 또는 비밀번호가 올바르지 않습니다.");
      return;
    }
    window.location.href = callbackUrl;
  }

  return (
    <div
      className="fixed inset-0 z-[100] flex items-center justify-center bg-black/70 backdrop-blur-sm px-4"
      role="dialog"
      aria-modal="true"
      onClick={onClose}
    >
      <div
        className="bezel-card w-full max-w-md"
        onClick={e => e.stopPropagation()}
      >
        <div className="bezel-inner p-8">
          <div className="flex justify-end mb-2">
            <button
              type="button"
              onClick={onClose}
              aria-label="닫기"
              className="w-8 h-8 -m-1 rounded-full text-surface-400 hover:text-white hover:bg-white/10 spring-transition flex items-center justify-center"
            >
              ✕
            </button>
          </div>
          <div className="flex items-center justify-center gap-2.5 mb-6">
            <div className="w-10 h-10 rounded-xl bg-gradient-to-br from-indigo-500 to-purple-500 flex items-center justify-center text-sm font-extrabold">
              ET
            </div>
            <span className="font-[var(--font-display)] font-bold text-lg tracking-tight">
              Easy Trainer
            </span>
          </div>
          <h2 className="font-[var(--font-display)] font-bold text-xl text-center mb-2">
            {title}
          </h2>
          <p className="text-surface-400 text-sm text-center mb-6">{description}</p>

          <button
            type="button"
            onClick={() => signIn("google", { callbackUrl })}
            className="w-full flex items-center justify-center gap-3 py-3 px-6 rounded-full bg-white text-surface-900 font-semibold text-sm hover:bg-surface-100 spring-transition cursor-pointer"
          >
            <svg className="w-5 h-5" viewBox="0 0 24 24">
              <path d="M22.56 12.25c0-.78-.07-1.53-.2-2.25H12v4.26h5.92a5.06 5.06 0 01-2.2 3.32v2.77h3.57c2.08-1.92 3.28-4.74 3.28-8.1z" fill="#4285F4" />
              <path d="M12 23c2.97 0 5.46-.98 7.28-2.66l-3.57-2.77c-.98.66-2.23 1.06-3.71 1.06-2.86 0-5.29-1.93-6.16-4.53H2.18v2.84C3.99 20.53 7.7 23 12 23z" fill="#34A853" />
              <path d="M5.84 14.09c-.22-.66-.35-1.36-.35-2.09s.13-1.43.35-2.09V7.07H2.18C1.43 8.55 1 10.22 1 12s.43 3.45 1.18 4.93l2.85-2.22.81-.62z" fill="#FBBC05" />
              <path d="M12 5.38c1.62 0 3.06.56 4.21 1.64l3.15-3.15C17.45 2.09 14.97 1 12 1 7.7 1 3.99 3.47 2.18 7.07l3.66 2.84c.87-2.6 3.3-4.53 6.16-4.53z" fill="#EA4335" />
            </svg>
            Google로 로그인
          </button>

          <div className="flex items-center gap-3 my-5">
            <span className="flex-1 h-px bg-white/10" />
            <span className="text-xs text-surface-500">또는</span>
            <span className="flex-1 h-px bg-white/10" />
          </div>

          <form onSubmit={handleCredentials} className="space-y-3">
            <input
              type="email"
              autoComplete="email"
              required
              value={email}
              onChange={e => setEmail(e.target.value)}
              placeholder="이메일"
              className="w-full px-4 py-2.5 rounded-lg bg-surface-900/60 border border-white/10 text-surface-100 text-sm placeholder:text-surface-600 focus:outline-none focus:border-indigo-500/60 focus:ring-2 focus:ring-indigo-500/20"
            />
            <input
              type="password"
              autoComplete="current-password"
              required
              value={password}
              onChange={e => setPassword(e.target.value)}
              placeholder="비밀번호"
              className="w-full px-4 py-2.5 rounded-lg bg-surface-900/60 border border-white/10 text-surface-100 text-sm placeholder:text-surface-600 focus:outline-none focus:border-indigo-500/60 focus:ring-2 focus:ring-indigo-500/20"
            />
            {error && (
              <p className="text-xs text-red-400 bg-red-500/10 border border-red-500/20 rounded-md px-3 py-2">
                {error}
              </p>
            )}
            <button
              type="submit"
              disabled={submitting || !email.trim() || !password}
              className="w-full py-2.5 rounded-full bg-indigo-500/20 hover:bg-indigo-500/30 border border-indigo-500/40 text-indigo-100 text-sm font-semibold spring-transition disabled:opacity-40 disabled:cursor-not-allowed cursor-pointer"
            >
              {submitting ? "로그인 중…" : "이메일로 로그인"}
            </button>
          </form>

          <p className="text-surface-600 text-xs mt-4 text-center">
            계정이 없다면 Google 로그인 후 약관 동의 절차로 자동 가입됩니다.
          </p>
        </div>
      </div>
    </div>
  );
}
