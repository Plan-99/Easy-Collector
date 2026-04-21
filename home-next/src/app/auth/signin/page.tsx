import { signIn } from "@/auth";
import Link from "next/link";

export default function SignInPage() {
  return (
    <div className="min-h-dvh flex items-center justify-center bg-surface-950 px-6">
      <div className="bezel-card w-full max-w-md">
        <div className="bezel-inner p-10 text-center">
          {/* Logo */}
          <div className="flex items-center justify-center gap-2.5 mb-8">
            <div className="w-10 h-10 rounded-xl bg-gradient-to-br from-indigo-500 to-purple-500 flex items-center justify-center text-sm font-extrabold">
              ET
            </div>
            <span className="font-[var(--font-display)] font-bold text-xl tracking-tight">
              Easy Trainer
            </span>
          </div>

          <h1 className="font-[var(--font-display)] font-bold text-2xl mb-2">
            시작하기
          </h1>
          <p className="text-surface-400 text-sm mb-10">
            Google 계정으로 로그인하면 라이선스 키가 자동 발급됩니다.
          </p>

          {/* Google Sign In */}
          <form
            action={async () => {
              "use server";
              await signIn("google", { redirectTo: "/dashboard" });
            }}
          >
            <button
              type="submit"
              className="w-full flex items-center justify-center gap-3 py-3.5 px-6 rounded-full bg-white text-surface-900 font-semibold text-sm hover:bg-surface-100 spring-transition cursor-pointer"
            >
              <svg className="w-5 h-5" viewBox="0 0 24 24">
                <path
                  d="M22.56 12.25c0-.78-.07-1.53-.2-2.25H12v4.26h5.92a5.06 5.06 0 01-2.2 3.32v2.77h3.57c2.08-1.92 3.28-4.74 3.28-8.1z"
                  fill="#4285F4"
                />
                <path
                  d="M12 23c2.97 0 5.46-.98 7.28-2.66l-3.57-2.77c-.98.66-2.23 1.06-3.71 1.06-2.86 0-5.29-1.93-6.16-4.53H2.18v2.84C3.99 20.53 7.7 23 12 23z"
                  fill="#34A853"
                />
                <path
                  d="M5.84 14.09c-.22-.66-.35-1.36-.35-2.09s.13-1.43.35-2.09V7.07H2.18C1.43 8.55 1 10.22 1 12s.43 3.45 1.18 4.93l2.85-2.22.81-.62z"
                  fill="#FBBC05"
                />
                <path
                  d="M12 5.38c1.62 0 3.06.56 4.21 1.64l3.15-3.15C17.45 2.09 14.97 1 12 1 7.7 1 3.99 3.47 2.18 7.07l3.66 2.84c.87-2.6 3.3-4.53 6.16-4.53z"
                  fill="#EA4335"
                />
              </svg>
              Google로 로그인
            </button>
          </form>

          <p className="text-surface-600 text-xs mt-8">
            로그인 시{" "}
            <a href="#" className="text-surface-400 underline">
              이용약관
            </a>{" "}
            및{" "}
            <a href="#" className="text-surface-400 underline">
              개인정보처리방침
            </a>
            에 동의합니다.
          </p>

          <Link
            href="/"
            className="inline-block mt-6 text-sm text-surface-500 hover:text-surface-300 spring-transition"
          >
            ← 홈으로 돌아가기
          </Link>
        </div>
      </div>
    </div>
  );
}
