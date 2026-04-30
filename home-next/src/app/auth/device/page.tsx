import { auth, signIn } from "@/auth";
import { prisma } from "@/lib/prisma";
import Link from "next/link";
import { redirect } from "next/navigation";
import DeviceConsentForm from "./DeviceConsentForm";

// Landing page for the launcher's device-auth flow.
// URL: /auth/device?code=<nonce>
//
// 1. If not signed in → show "Sign in with Google" with the code preserved in
//    the redirect target.
// 2. If signed in → look up the nonce, show machine info + 동의/거절 buttons.
//    Submission goes to /api/device-auth/approve (handled by the client form).

type Props = {
  searchParams: Promise<{ code?: string }>;
};

export default async function DeviceAuthPage({ searchParams }: Props) {
  const { code } = await searchParams;
  const nonce = (code || "").trim();

  if (!nonce) {
    return (
      <div className="min-h-dvh flex items-center justify-center bg-surface-950 px-6">
        <div className="bezel-card w-full max-w-md">
          <div className="bezel-inner p-10 text-center">
            <h1 className="font-bold text-xl mb-3">유효하지 않은 링크</h1>
            <p className="text-surface-400 text-sm mb-6">
              런처에서 표시한 링크를 다시 확인해 주세요.
            </p>
            <Link href="/" className="text-indigo-400 hover:text-indigo-300 text-sm">
              홈으로
            </Link>
          </div>
        </div>
      </div>
    );
  }

  const session = await auth();

  if (!session?.user?.id) {
    // Bounce through Google sign-in, returning to this same URL on success.
    const callbackUrl = `/auth/device?code=${encodeURIComponent(nonce)}`;
    return (
      <div className="min-h-dvh flex items-center justify-center bg-surface-950 px-6">
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
            <h1 className="font-bold text-xl mb-3">기기 등록</h1>
            <p className="text-surface-400 text-sm mb-8">
              런처를 사용하려면 Google 계정으로 로그인해 주세요.
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
          </div>
        </div>
      </div>
    );
  }

  // Onboarding gate: a launcher binding requires the marketing+legal profile.
  const me = await prisma.user.findUnique({
    where: { id: session.user.id },
    select: {
      organization: true,
      department: true,
      jobRole: true,
      termsAcceptedAt: true,
      privacyAcceptedAt: true,
      refundAcceptedAt: true,
    },
  });
  const onboarded =
    !!me?.organization &&
    !!me.department &&
    !!me.jobRole &&
    !!me.termsAcceptedAt &&
    !!me.privacyAcceptedAt &&
    !!me.refundAcceptedAt;
  if (!onboarded) {
    const next = `/auth/device?code=${encodeURIComponent(nonce)}`;
    redirect(`/onboarding?next=${encodeURIComponent(next)}`);
  }

  // Signed in. Look up the request + check whether this user already has a
  // device bound to a *different* machine — that's the "이미 사용중인 기기"
  // case and we surface it before the user even clicks 등록.
  const reqRow = await prisma.deviceAuthRequest.findUnique({ where: { nonce } });
  const existingDevice = await prisma.device.findFirst({
    where: { userId: session.user.id, active: true },
    select: { machineId: true, hostname: true, lastSeenAt: true },
  });

  let state:
    | "missing"
    | "expired"
    | "consumed"
    | "rejected"
    | "approved"
    | "blocked"
    | "ready" = "ready";
  if (!reqRow) state = "missing";
  else if (reqRow.status === "EXPIRED" || reqRow.expiresAt < new Date()) state = "expired";
  else if (reqRow.status === "CONSUMED") state = "consumed";
  else if (reqRow.status === "REJECTED") state = "rejected";
  else if (reqRow.status === "APPROVED") state = "approved";
  else if (
    existingDevice &&
    reqRow &&
    existingDevice.machineId !== reqRow.machineId
  )
    state = "blocked";

  return (
    <div className="min-h-dvh flex items-center justify-center bg-surface-950 px-6">
      <div className="bezel-card w-full max-w-lg">
        <div className="bezel-inner p-10">
          <div className="flex items-center gap-3 mb-8">
            <div className="w-10 h-10 rounded-xl bg-gradient-to-br from-indigo-500 to-purple-500 flex items-center justify-center text-sm font-extrabold">
              ET
            </div>
            <span className="font-[var(--font-display)] font-bold text-xl tracking-tight">
              기기 등록
            </span>
          </div>

          <p className="text-surface-300 mb-6 text-sm">
            <span className="text-surface-500">계정:</span>{" "}
            <span className="font-medium">{session.user.email}</span>
          </p>

          {state === "missing" && (
            <Notice tone="error" title="유효하지 않은 코드">
              런처에서 표시된 링크가 만료되었거나 잘못되었습니다. 런처에서 다시 시작해 주세요.
            </Notice>
          )}

          {state === "expired" && (
            <Notice tone="error" title="시간이 만료되었습니다">
              인증 코드는 10분 동안만 유효합니다. 런처로 돌아가 다시 시도해 주세요.
            </Notice>
          )}

          {state === "consumed" && (
            <Notice tone="warn" title="이미 사용된 코드">
              이 인증 코드는 이미 사용되었습니다. 런처에서 다시 시작해 주세요.
            </Notice>
          )}

          {state === "rejected" && (
            <Notice tone="error" title="거절된 요청">
              이전에 거절된 요청입니다. 런처에서 다시 시작해 주세요.
            </Notice>
          )}

          {state === "approved" && (
            <Notice tone="ok" title="이미 승인되었습니다">
              런처가 자동으로 진행을 이어갑니다. 이 창을 닫으셔도 됩니다.
            </Notice>
          )}

          {state === "blocked" && existingDevice && reqRow && (
            <Notice tone="error" title="이미 사용중인 기기가 있습니다">
              <div className="space-y-2 text-sm">
                <p>
                  현재 계정에는 다른 PC가 이미 등록되어 있어 새로운 기기를 추가할 수 없습니다.
                </p>
                <ul className="text-surface-400 text-xs space-y-1 mt-3">
                  <li>
                    기존 기기:{" "}
                    <code className="text-surface-300">
                      {existingDevice.hostname || existingDevice.machineId.slice(0, 12) + "…"}
                    </code>
                  </li>
                  <li>
                    새 기기:{" "}
                    <code className="text-surface-300">
                      {reqRow.hostname || reqRow.machineId.slice(0, 12) + "…"}
                    </code>
                  </li>
                </ul>
                <p className="mt-4">
                  <Link href="/dashboard" className="text-indigo-400 hover:text-indigo-300 underline">
                    대시보드에서 기존 기기 등록 해제 →
                  </Link>
                </p>
              </div>
            </Notice>
          )}

          {state === "ready" && reqRow && (
            <DeviceConsentForm
              nonce={nonce}
              machineId={reqRow.machineId}
              hostname={reqRow.hostname}
              os={reqRow.os}
            />
          )}
        </div>
      </div>
    </div>
  );
}

function Notice({
  tone,
  title,
  children,
}: {
  tone: "ok" | "warn" | "error";
  title: string;
  children: React.ReactNode;
}) {
  const color =
    tone === "ok"
      ? "border-emerald-500/30 bg-emerald-500/5"
      : tone === "warn"
        ? "border-amber-500/30 bg-amber-500/5"
        : "border-red-500/30 bg-red-500/5";
  return (
    <div className={`rounded-xl border ${color} p-5`}>
      <p className="font-semibold mb-2">{title}</p>
      <div className="text-surface-300 text-sm">{children}</div>
    </div>
  );
}
