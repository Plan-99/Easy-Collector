import { auth, signOut } from "@/auth";
import { prisma } from "@/lib/prisma";
import { redirect } from "next/navigation";
import Link from "next/link";
import Footer from "@/components/Footer";
import UnbindDeviceButton from "./UnbindDeviceButton";
import RefundButton from "./RefundButton";

const REFUND_WINDOW_MS = 7 * 24 * 60 * 60 * 1000;

function formatKrw(amountKrw: number) {
  return new Intl.NumberFormat("ko-KR", {
    style: "currency",
    currency: "KRW",
    maximumFractionDigits: 0,
  }).format(amountKrw);
}

export default async function DashboardPage() {
  const session = await auth();
  if (!session?.user?.id) redirect("/auth/signin");

  const user = await prisma.user.findUnique({
    where: { id: session.user.id },
    include: {
      devices: { orderBy: { lastSeenAt: "desc" } },
      entitlements: {
        include: { module: true },
        orderBy: { grantedAt: "desc" },
      },
      payments: { orderBy: { createdAt: "desc" }, take: 20 },
      billingKeys: { where: { active: true }, orderBy: { createdAt: "desc" } },
    },
  });

  if (!user) redirect("/auth/signin");

  const onboarded =
    !!user.organization &&
    !!user.department &&
    !!user.jobRole &&
    !!user.termsAcceptedAt &&
    !!user.privacyAcceptedAt &&
    !!user.refundAcceptedAt;
  if (!onboarded) redirect("/onboarding?next=/dashboard");

  const activeDevice = user.devices.find(d => d.active) || null;
  const paidEntitlements = user.entitlements.filter(e => e.paymentId !== null);

  return (
    <div className="min-h-dvh bg-surface-950 pt-16 flex flex-col">
      <div className="flex-1 px-6">
        <div className="max-w-3xl mx-auto py-16">
        <div className="flex items-center justify-between mb-12">
          <div className="flex items-center gap-3">
            <div className="w-10 h-10 rounded-xl bg-gradient-to-br from-indigo-500 to-purple-500 flex items-center justify-center text-sm font-extrabold">
              ET
            </div>
            <span className="font-[var(--font-display)] font-bold text-xl tracking-tight">
              내 정보
            </span>
          </div>
          <div className="flex items-center gap-4">
            <Link href="/" className="text-sm text-surface-400 hover:text-white spring-transition">
              홈
            </Link>
            <form
              action={async () => {
                "use server";
                await signOut({ redirectTo: "/" });
              }}
            >
              <button
                type="submit"
                className="text-sm text-surface-400 hover:text-white spring-transition"
              >
                로그아웃
              </button>
            </form>
          </div>
        </div>

        {/* Profile */}
        <div className="bezel-card mb-8">
          <div className="bezel-inner p-8">
            <div className="flex items-center gap-5">
              {user.image ? (
                // eslint-disable-next-line @next/next/no-img-element
                <img src={user.image} alt={user.name || ""} className="w-14 h-14 rounded-full" />
              ) : (
                <div className="w-14 h-14 rounded-full bg-gradient-to-br from-indigo-500 to-purple-500 flex items-center justify-center font-bold text-lg">
                  {(user.name || user.email)?.[0]?.toUpperCase()}
                </div>
              )}
              <div className="min-w-0">
                <h2 className="font-bold text-lg truncate">{user.name}</h2>
                <p className="text-surface-400 text-sm truncate">{user.email}</p>
                {(user.organization || user.department || user.jobRole) && (
                  <p className="text-surface-500 text-xs mt-1 truncate">
                    {[user.organization, user.department, user.jobRole]
                      .filter(Boolean)
                      .join(" · ")}
                  </p>
                )}
              </div>
              <div className="ml-auto">
                <span
                  className={`px-3 py-1 rounded-full text-xs font-semibold ${
                    user.plan === "business"
                      ? "bg-indigo-500/15 text-indigo-300"
                      : "bg-surface-800 text-surface-400"
                  }`}
                >
                  {user.plan === "business" ? "Business" : "Free"} Plan
                </span>
              </div>
            </div>
          </div>
        </div>

        {/* Connected device */}
        <div className="bezel-card mb-8">
          <div className="bezel-inner p-8">
            <h3 className="font-[var(--font-display)] font-bold text-lg mb-1">연결된 기기</h3>
            <p className="text-surface-500 text-sm mb-6">
              한 계정당 한 대의 PC만 활성 상태로 유지됩니다. 다른 PC에서 사용하려면 등록을 해제하세요.
            </p>

            {activeDevice ? (
              <div className="rounded-xl bg-surface-900/60 border border-white/5 p-5">
                <div className="flex items-start justify-between gap-4">
                  <div className="space-y-1.5 text-sm">
                    <p className="font-semibold text-surface-100">
                      {activeDevice.hostname || "(호스트명 미상)"}
                    </p>
                    <p className="text-surface-400 text-xs">
                      {activeDevice.os || "OS 미상"} · 머신ID{" "}
                      <code className="text-surface-300">{activeDevice.machineId.slice(0, 12)}…</code>
                    </p>
                    <p className="text-surface-500 text-xs">
                      최근 접속:{" "}
                      {new Date(activeDevice.lastSeenAt).toLocaleString("ko-KR")}
                    </p>
                  </div>
                  <UnbindDeviceButton deviceId={activeDevice.id} />
                </div>
              </div>
            ) : (
              <p className="text-surface-500 text-sm">
                아직 연결된 기기가 없습니다. Easy Trainer 런처에서 Google 로그인을 진행하면 자동으로 등록됩니다.
              </p>
            )}
          </div>
        </div>

        {/* Owned modules */}
        <div className="bezel-card mb-8">
          <div className="bezel-inner p-8">
            <h3 className="font-[var(--font-display)] font-bold text-lg mb-1">보유 모듈</h3>
            <p className="text-surface-500 text-sm mb-6">
              결제하여 영구 보유 중인 모듈 목록입니다. 무료 모듈은 기본 제공되어 표시되지 않습니다.
            </p>

            {paidEntitlements.length > 0 ? (
              <ul className="divide-y divide-white/5">
                {paidEntitlements.map(e => (
                  <li key={e.id} className="py-3 flex items-center justify-between text-sm">
                    <div>
                      <p className="font-medium text-surface-100">{e.module.name}</p>
                      <p className="text-surface-500 text-xs">{e.module.id}</p>
                    </div>
                    <p className="text-surface-400 text-xs">
                      {new Date(e.grantedAt).toLocaleDateString("ko-KR")}
                    </p>
                  </li>
                ))}
              </ul>
            ) : (
              <p className="text-surface-500 text-sm">
                구매한 모듈이 없습니다. 런처에서 원하는 모듈의 설치 버튼을 눌러 결제하세요.
              </p>
            )}
          </div>
        </div>

        {/* Payment history */}
        <div className="bezel-card mb-8">
          <div className="bezel-inner p-8">
            <h3 className="font-[var(--font-display)] font-bold text-lg mb-1">결제 내역</h3>
            <p className="text-surface-500 text-sm mb-6">최근 20건</p>

            {user.payments.length > 0 ? (
              <ul className="divide-y divide-white/5">
                {user.payments.map(p => {
                  const refundable =
                    p.status === "PAID" &&
                    !!p.paidAt &&
                    Date.now() - new Date(p.paidAt).getTime() < REFUND_WINDOW_MS;
                  return (
                    <li key={p.id} className="py-3 flex items-center justify-between text-sm gap-4">
                      <div className="min-w-0 flex-1">
                        <p className="font-medium text-surface-100 truncate">
                          {p.moduleId || "기타"}{" "}
                          <span className="text-surface-500 text-xs ml-2">{p.method || ""}</span>
                        </p>
                        <p className="text-surface-500 text-xs">
                          {new Date(p.createdAt).toLocaleString("ko-KR")}
                        </p>
                      </div>
                      <div className="text-right shrink-0">
                        <p className="font-mono">{formatKrw(p.amountKrw)}</p>
                        <div className="flex items-center justify-end gap-3">
                          <p
                            className={`text-xs ${
                              p.status === "PAID"
                                ? "text-emerald-400"
                                : p.status === "REFUNDED" || p.status === "CANCELLED"
                                  ? "text-amber-400"
                                  : p.status === "FAILED"
                                    ? "text-red-400"
                                    : "text-surface-500"
                            }`}
                          >
                            {p.status}
                          </p>
                          {refundable && <RefundButton paymentId={p.id} />}
                        </div>
                      </div>
                    </li>
                  );
                })}
              </ul>
            ) : (
              <p className="text-surface-500 text-sm">결제 내역이 없습니다.</p>
            )}
          </div>
        </div>

        {/* Saved cards */}
        <div className="bezel-card mb-8">
          <div className="bezel-inner p-8">
            <h3 className="font-[var(--font-display)] font-bold text-lg mb-1">저장된 결제 수단</h3>
            <p className="text-surface-500 text-sm mb-6">
              다음 결제부터 추가 인증 없이 사용할 수 있습니다.
            </p>

            {user.billingKeys.length > 0 ? (
              <ul className="divide-y divide-white/5">
                {user.billingKeys.map(b => (
                  <li key={b.id} className="py-3 flex items-center justify-between text-sm">
                    <div>
                      <p className="font-medium text-surface-100">
                        {b.cardName || "카드"} ····{b.cardLast4 || "????"}
                        {b.isDefault && (
                          <span className="ml-2 text-[10px] px-1.5 py-0.5 rounded bg-indigo-500/20 text-indigo-300 font-semibold">
                            기본
                          </span>
                        )}
                      </p>
                      <p className="text-surface-500 text-xs">
                        등록일 {new Date(b.createdAt).toLocaleDateString("ko-KR")}
                      </p>
                    </div>
                  </li>
                ))}
              </ul>
            ) : (
              <p className="text-surface-500 text-sm">
                저장된 결제 수단이 없습니다. 첫 결제 시 카드 저장에 동의하면 다음 결제부터 표시됩니다.
              </p>
            )}
          </div>
        </div>

        {/* Download */}
        <div className="bezel-card">
          <div className="bezel-inner p-8">
            <h3 className="font-[var(--font-display)] font-bold text-lg mb-1">다운로드</h3>
            <p className="text-surface-500 text-sm mb-6">Ubuntu 22.04 이상, NVIDIA GPU 필수</p>

            <a
              href="/api/download/deb"
              className="btn-pill bg-gradient-to-r from-indigo-500 to-purple-500 text-white shadow-lg shadow-indigo-500/20 hover:shadow-indigo-500/35 hover:-translate-y-0.5 text-sm"
            >
              Easy Trainer .deb 다운로드
              <span className="icon-circle bg-white/20">↓</span>
            </a>
          </div>
        </div>
        </div>
      </div>
      <Footer />
    </div>
  );
}
