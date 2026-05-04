import { auth, signIn, signOut } from "@/auth";
import { prisma } from "@/lib/prisma";
import { redirect } from "next/navigation";
import Link from "next/link";
import Footer from "@/components/Footer";
import CheckoutClient from "./CheckoutClient";
import { PORTONE_STORE_ID, PORTONE_CHANNEL_KEY } from "@/lib/portone";

export const dynamic = "force-dynamic";

function formatKrw(amountKrw: number) {
  return new Intl.NumberFormat("ko-KR", {
    style: "currency",
    currency: "KRW",
    maximumFractionDigits: 0,
  }).format(amountKrw);
}

export default async function CheckoutPage({
  params,
  searchParams,
}: {
  params: Promise<{ moduleId: string }>;
  searchParams: Promise<{ u?: string; paymentId?: string; code?: string; message?: string }>;
}) {
  const { moduleId } = await params;
  const sp = await searchParams;
  const expectedUserId = typeof sp.u === "string" ? sp.u.trim() : "";
  const session = await auth();

  if (!session?.user?.id) {
    const qs = expectedUserId ? `?u=${encodeURIComponent(expectedUserId)}` : "";
    const callbackUrl = `/checkout/${encodeURIComponent(moduleId)}${qs}`;
    return (
      <div className="min-h-dvh flex flex-col bg-surface-950">
        <div className="flex-1 flex items-center justify-center px-6">
          <div className="bezel-card w-full max-w-md">
            <div className="bezel-inner p-10 text-center">
              <h1 className="font-bold text-xl mb-3">로그인이 필요합니다</h1>
              <p className="text-surface-400 text-sm mb-8">
                결제를 진행하려면 Google 계정으로 로그인해 주세요.
              </p>
              <form
                action={async () => {
                  "use server";
                  await signIn("google", { redirectTo: callbackUrl });
                }}
              >
                <button className="w-full py-3 rounded-full bg-white text-surface-900 font-semibold text-sm hover:bg-surface-100 spring-transition cursor-pointer">
                  Google로 로그인
                </button>
              </form>
            </div>
          </div>
        </div>
        <Footer />
      </div>
    );
  }

  // Onboarding gate — same as dashboard. PortOne KYC needs profile.
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
    redirect(`/onboarding?next=${encodeURIComponent(`/checkout/${moduleId}`)}`);
  }

  // Launcher passes the registered user.id as ?u=…. If the browser is
  // signed in as a different account, surface a clear error before the
  // user wastes a payment on the wrong account.
  if (expectedUserId && expectedUserId !== session.user.id) {
    const me2 = await prisma.user.findUnique({
      where: { id: session.user.id },
      select: { email: true },
    });
    return (
      <CheckoutShell>
        <Notice tone="error" title="다른 계정으로 로그인되어 있습니다">
          <p>
            런처에 등록된 계정과 다른 계정({me2?.email || session.user.email})으로
            브라우저에 로그인되어 있어 결제를 진행할 수 없습니다.
          </p>
          <p className="mt-3">현재 계정에서 로그아웃한 뒤 런처와 동일한 계정으로 다시 로그인해 주세요.</p>
          <form
            action={async () => {
              "use server";
              await signOut({
                redirectTo: `/checkout/${encodeURIComponent(moduleId)}?u=${encodeURIComponent(expectedUserId)}`,
              });
            }}
            className="mt-5"
          >
            <button
              type="submit"
              className="px-5 py-2.5 rounded-full bg-red-500/20 hover:bg-red-500/30 border border-red-500/30 text-red-200 text-sm font-semibold spring-transition cursor-pointer"
            >
              로그아웃하고 다시 시도
            </button>
          </form>
        </Notice>
      </CheckoutShell>
    );
  }

  const moduleRow = await prisma.module.findUnique({ where: { id: moduleId } });

  if (!moduleRow || !moduleRow.active) {
    return (
      <CheckoutShell>
        <Notice tone="error" title="존재하지 않는 모듈">
          요청하신 모듈을 찾을 수 없습니다.
          <p className="mt-3">
            <Link href="/dashboard" className="text-indigo-400 hover:text-indigo-300 underline">
              대시보드로 이동 →
            </Link>
          </p>
        </Notice>
      </CheckoutShell>
    );
  }

  // Already owned (paid Entitlement OR free module)?
  if (moduleRow.priceKrw === 0) {
    return (
      <CheckoutShell>
        <Notice tone="ok" title="무료 모듈입니다">
          별도 결제 없이 런처에서 바로 설치할 수 있습니다.
          <p className="mt-3">
            <Link href="/dashboard" className="text-indigo-400 hover:text-indigo-300 underline">
              대시보드로 이동 →
            </Link>
          </p>
        </Notice>
      </CheckoutShell>
    );
  }

  const owned = await prisma.entitlement.findUnique({
    where: { userId_moduleId: { userId: session.user.id, moduleId } },
  });
  if (owned) {
    return (
      <CheckoutShell>
        <Notice tone="ok" title="이미 보유 중인 모듈">
          이 모듈은 이전에 결제 완료되었습니다. 런처에서 바로 설치할 수 있습니다.
          <p className="mt-3">
            <Link href="/dashboard" className="text-indigo-400 hover:text-indigo-300 underline">
              대시보드로 이동 →
            </Link>
          </p>
        </Notice>
      </CheckoutShell>
    );
  }

  const configMissing = !PORTONE_STORE_ID || !PORTONE_CHANNEL_KEY;

  return (
    <CheckoutShell>
      <div className="rounded-xl bg-surface-900/60 border border-white/5 p-6 mb-6">
        <p className="text-surface-500 text-xs uppercase tracking-wide mb-2">
          {moduleRow.category}
        </p>
        <h2 className="font-[var(--font-display)] font-bold text-xl mb-2">
          {moduleRow.name}
        </h2>
        {moduleRow.description && (
          <p className="text-surface-400 text-sm mb-4 leading-relaxed">
            {moduleRow.description}
          </p>
        )}
        <p className="font-mono text-2xl">{formatKrw(moduleRow.priceKrw)}</p>
      </div>

      {configMissing ? (
        <Notice tone="warn" title="결제 설정이 필요합니다">
          서버에 PortOne 환경변수(`PORTONE_STORE_ID`, `PORTONE_CHANNEL_KEY`)가
          누락되어 있습니다. 관리자에게 문의해 주세요.
        </Notice>
      ) : (
        <CheckoutClient
          moduleId={moduleRow.id}
          moduleName={moduleRow.name}
          priceKrw={moduleRow.priceKrw}
          buyerEmail={session.user.email || ""}
          buyerName={session.user.name || ""}
          // PortOne PG (갤럭시아) requires customerId max 20 chars; user.id is
          // a 25-char cuid so we truncate. Stable per user → safe for billing
          // history correlation.
          customerId={session.user.id.slice(0, 20)}
          storeId={PORTONE_STORE_ID}
          channelKey={PORTONE_CHANNEL_KEY}
        />
      )}

      <p className="text-surface-500 text-xs mt-6 text-center leading-relaxed">
        결제 진행 시{" "}
        <Link href="/dashboard" className="underline hover:text-surface-300">
          이용약관·환불정책
        </Link>
        에 동의한 것으로 간주됩니다. 결제 후 7일 이내, 모듈을 다운로드하지 않은 경우에 한해
        전액 환불 가능합니다.
      </p>
    </CheckoutShell>
  );
}

function CheckoutShell({ children }: { children: React.ReactNode }) {
  return (
    <div className="min-h-dvh flex flex-col bg-surface-950">
      <div className="flex-1 px-6 py-16 flex items-center justify-center">
        <div className="w-full max-w-lg">
          <div className="flex items-center gap-2.5 mb-8">
            <div className="w-10 h-10 rounded-xl bg-gradient-to-br from-indigo-500 to-purple-500 flex items-center justify-center text-sm font-extrabold">
              ET
            </div>
            <span className="font-[var(--font-display)] font-bold text-xl tracking-tight">
              결제
            </span>
          </div>

          <div className="bezel-card">
            <div className="bezel-inner p-8">{children}</div>
          </div>
        </div>
      </div>
      <Footer />
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
