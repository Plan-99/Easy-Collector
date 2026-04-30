import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";
import { redirect } from "next/navigation";
import UnbindDeviceButton from "./UnbindDeviceButton";
import RefundButton from "./RefundButton";
import PasswordCard from "./PasswordCard";

const REFUND_WINDOW_MS = 7 * 24 * 60 * 60 * 1000;

function formatKrw(amountKrw: number) {
  return new Intl.NumberFormat("ko-KR", {
    style: "currency",
    currency: "KRW",
    maximumFractionDigits: 0,
  }).format(amountKrw);
}

function PlanBadge({ plan }: { plan: string }) {
  const cls =
    plan === "business"
      ? "bg-amber-500/15 text-amber-300"
      : plan === "unlimited"
        ? "bg-indigo-500/15 text-indigo-300"
        : "bg-surface-800 text-surface-400";
  const label =
    plan === "business" ? "Business" : plan === "unlimited" ? "Unlimited" : "Free";
  return (
    <span className={`px-3 py-1 rounded-full text-xs font-semibold ${cls}`}>
      {label} Plan
    </span>
  );
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

  const activeDevices = user.devices.filter(d => d.active);
  const isUnlimited = user.plan === "unlimited" || user.plan === "business";
  const paidEntitlements = user.entitlements.filter(e => e.paymentId !== null);
  const hasPassword = !!user.passwordHash;

  return (
    <>
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
              <PlanBadge plan={user.plan} />
            </div>
          </div>
        </div>
      </div>

      {/* Password */}
      <PasswordCard hasPassword={hasPassword} />

      {/* Connected devices */}
      <div className="bezel-card mb-8">
        <div className="bezel-inner p-8">
          <h3 className="font-[var(--font-display)] font-bold text-lg mb-1">
            연결된 기기 {activeDevices.length > 0 && (
              <span className="text-surface-500 text-sm ml-1">({activeDevices.length})</span>
            )}
          </h3>
          <p className="text-surface-500 text-sm mb-6">
            {isUnlimited
              ? "Unlimited 플랜은 여러 PC에서 동시에 사용할 수 있습니다. 더 이상 쓰지 않는 기기는 등록 해제하세요."
              : "Free 플랜은 한 대의 PC만 활성 상태로 유지됩니다. 다른 PC에서 사용하려면 등록을 해제하세요."}
          </p>
          {activeDevices.length > 0 ? (
            <div className="space-y-3">
              {activeDevices.map(d => (
                <div key={d.id} className="rounded-xl bg-surface-900/60 border border-white/5 p-5">
                  <div className="flex items-start justify-between gap-4">
                    <div className="space-y-1.5 text-sm min-w-0">
                      <p className="font-semibold text-surface-100 truncate">
                        {d.hostname || "(호스트명 미상)"}
                      </p>
                      <p className="text-surface-400 text-xs">
                        {d.os || "OS 미상"} · 머신ID{" "}
                        <code className="text-surface-300">{d.machineId.slice(0, 12)}…</code>
                      </p>
                      <p className="text-surface-500 text-xs">
                        최근 접속: {new Date(d.lastSeenAt).toLocaleString("ko-KR")}
                      </p>
                    </div>
                    <UnbindDeviceButton deviceId={d.id} />
                  </div>
                </div>
              ))}
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
              구매한 모듈이 없습니다. <a href="/dashboard/modules" className="text-indigo-400 hover:text-indigo-300 underline">모듈 탭</a>에서 둘러보세요.
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
    </>
  );
}
