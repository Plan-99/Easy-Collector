import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";
import { redirect } from "next/navigation";
import Link from "next/link";
import CancelPendingButton from "./CancelPendingButton";

function formatKrw(amountKrw: number) {
  return new Intl.NumberFormat("ko-KR", {
    style: "currency",
    currency: "KRW",
    maximumFractionDigits: 0,
  }).format(amountKrw);
}

const STATUS_LABEL: Record<string, string> = {
  PENDING: "결제 대기",
  PAID: "결제 완료 / 출고 준비",
  SHIPPED: "배송 중",
  DELIVERED: "배송 완료",
  CANCELLED: "취소",
  REFUNDED: "환불",
};

export default async function DashboardOrdersPage() {
  const session = await auth();
  if (!session?.user?.id) redirect("/auth/signin");

  const orders = await prisma.order.findMany({
    where: { userId: session.user.id },
    include: { items: true, shipment: true },
    orderBy: { createdAt: "desc" },
  });

  return (
    <>
      <h2 className="font-[var(--font-display)] font-bold text-xl mb-2">하드웨어 주문</h2>
      <p className="text-surface-500 text-sm mb-6">
        스토어에서 주문한 하드웨어 내역과 배송 상태를 확인할 수 있습니다.
      </p>

      {orders.length === 0 ? (
        <div className="bezel-card">
          <div className="bezel-inner p-8 text-center text-surface-500 text-sm">
            아직 주문 내역이 없습니다.{" "}
            <Link href="/store" className="text-indigo-400 hover:text-indigo-300 underline">
              스토어로 이동
            </Link>
          </div>
        </div>
      ) : (
        <div className="space-y-3">
          {orders.map(o => (
            <Link
              key={o.id}
              href={`/dashboard/orders/${o.id}`}
              className="bezel-card spring-transition hover:-translate-y-0.5 block"
            >
              <div className="bezel-inner p-5">
                <div className="flex items-center justify-between mb-2">
                  <span className="text-xs text-surface-500">
                    {new Date(o.createdAt).toLocaleString("ko-KR")}
                  </span>
                  <span className="text-xs font-semibold px-2 py-0.5 rounded bg-surface-800 text-surface-300">
                    {STATUS_LABEL[o.status] || o.status}
                  </span>
                </div>
                <p className="font-medium text-surface-100">
                  {o.items
                    .map(i =>
                      `${i.productName}${i.variantName ? ` (${i.variantName})` : ""} × ${i.quantity}`
                    )
                    .join(", ")}
                </p>
                <div className="flex items-center justify-between mt-2 text-xs gap-3">
                  <span className="text-surface-500 truncate">
                    {o.shipment
                      ? `${o.shipment.courier} ${o.shipment.trackingNumber}`
                      : o.status === "PAID"
                        ? "출고 대기 중"
                        : "—"}
                  </span>
                  <div className="flex items-center gap-3 shrink-0">
                    {o.status === "PENDING" && (
                      <CancelPendingButton orderId={o.id} variant="inline" />
                    )}
                    <span className="font-mono">{formatKrw(o.totalKrw)}</span>
                  </div>
                </div>
              </div>
            </Link>
          ))}
        </div>
      )}
    </>
  );
}
