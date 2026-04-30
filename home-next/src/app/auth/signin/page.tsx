import { signIn } from "@/auth";
import Link from "next/link";
import Footer from "@/components/Footer";
import CredentialsForm from "./CredentialsForm";

export default async function SignInPage({
  searchParams,
}: {
  searchParams: Promise<{ callbackUrl?: string }>;
}) {
  const sp = await searchParams;
  const rawNext = typeof sp.callbackUrl === "string" ? sp.callbackUrl : "";
  const callbackUrl = rawNext.startsWith("/") ? rawNext : "/onboarding";

  return (
    <div className="min-h-dvh flex flex-col bg-surface-950">
      <div className="flex-1 flex items-center justify-center px-6 py-16">
        <div className="bezel-card w-full max-w-md">
          <div className="bezel-inner p-10 text-center">
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
            <p className="text-surface-400 text-sm mb-8">
              Google로 가입한 후 내정보에서 비밀번호를 설정하면 이메일 로그인도 사용할 수 있습니다.
            </p>

            <form
              action={async () => {
                "use server";
                await signIn("google", { redirectTo: callbackUrl });
              }}
            >
              <button
                type="submit"
                className="w-full flex items-center justify-center gap-3 py-3.5 px-6 rounded-full bg-white text-surface-900 font-semibold text-sm hover:bg-surface-100 spring-transition cursor-pointer"
              >
                <svg className="w-5 h-5" viewBox="0 0 24 24">
                  <path d="M22.56 12.25c0-.78-.07-1.53-.2-2.25H12v4.26h5.92a5.06 5.06 0 01-2.2 3.32v2.77h3.57c2.08-1.92 3.28-4.74 3.28-8.1z" fill="#4285F4" />
                  <path d="M12 23c2.97 0 5.46-.98 7.28-2.66l-3.57-2.77c-.98.66-2.23 1.06-3.71 1.06-2.86 0-5.29-1.93-6.16-4.53H2.18v2.84C3.99 20.53 7.7 23 12 23z" fill="#34A853" />
                  <path d="M5.84 14.09c-.22-.66-.35-1.36-.35-2.09s.13-1.43.35-2.09V7.07H2.18C1.43 8.55 1 10.22 1 12s.43 3.45 1.18 4.93l2.85-2.22.81-.62z" fill="#FBBC05" />
                  <path d="M12 5.38c1.62 0 3.06.56 4.21 1.64l3.15-3.15C17.45 2.09 14.97 1 12 1 7.7 1 3.99 3.47 2.18 7.07l3.66 2.84c.87-2.6 3.3-4.53 6.16-4.53z" fill="#EA4335" />
                </svg>
                Google로 로그인
              </button>
            </form>

            <div className="flex items-center gap-3 my-6">
              <span className="flex-1 h-px bg-white/10" />
              <span className="text-xs text-surface-500">또는</span>
              <span className="flex-1 h-px bg-white/10" />
            </div>

            <CredentialsForm callbackUrl={callbackUrl} />

            <p className="text-surface-600 text-xs mt-6">
              가입 완료 단계에서 이용약관·개인정보처리방침·환불 정책에 동의하게 됩니다.
            </p>

            <Link
              href="/"
              className="inline-block mt-4 text-sm text-surface-500 hover:text-surface-300 spring-transition"
            >
              ← 홈으로 돌아가기
            </Link>
          </div>
        </div>
      </div>
      <Footer />
    </div>
  );
}
