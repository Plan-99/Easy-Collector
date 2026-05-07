import { prisma } from "@/lib/prisma";
import Link from "next/link";

function formatKrw(amountKrw: number) {
  return new Intl.NumberFormat("ko-KR", {
    style: "currency",
    currency: "KRW",
    maximumFractionDigits: 0,
  }).format(amountKrw);
}

function statusBadge(status: string) {
  const cls =
    status === "PAID"
      ? "bg-emerald-500/15 text-emerald-300"
      : status === "SHIPPED"
        ? "bg-indigo-500/15 text-indigo-300"
        : status === "DELIVERED"
          ? "bg-sky-500/15 text-sky-300"
          : status === "CANCELLED" || status === "REFUNDED"
            ? "bg-amber-500/15 text-amber-300"
            : status === "PENDING"
              ? "bg-surface-800 text-surface-400"
              : "bg-red-500/15 text-red-300";
  return cls;
}

export default async function AdminOrdersPage() {
  const orders = await prisma.order.findMany({
    include: {
      user: { select: { email: true, name: true } },
      items: true,
      shipment: true,
    },
    orderBy: { createdAt: "desc" },
    take: 100,
  });

  return (
    <>
      <p className="text-surface-400 text-sm mb-4">최근 100건</p>
      <div className="bezel-card">
        <div className="bezel-inner p-0 overflow-hidden">
          <table className="w-full text-sm">
            <thead>
              <tr className="border-b border-white/10 text-left text-surface-400">
                <th className="px-5 py-3 font-medium">주문일시</th>
                <th className="px-5 py-3 font-medium">주문번호</th>
                <th className="px-5 py-3 font-medium">사용자</th>
                <th className="px-5 py-3 font-medium">상품</th>
                <th className="px-5 py-3 font-medium text-right">금액</th>
                <th className="px-5 py-3 font-medium text-center">상태</th>
                <th className="px-5 py-3 font-medium text-center">송장</th>
                <th className="px-5 py-3 font-medium text-right">상세</th>
              </tr>
            </thead>
            <tbody>
              {orders.map(o => (
                <tr key={o.id} className="border-b border-white/5 hover:bg-white/[0.02]">
                  <td className="px-5 py-3 text-xs text-surface-400">
                    {new Date(o.createdAt).toLocaleString("ko-KR")}
                  </td>
                  <td className="px-5 py-3">
                    <code className="text-xs text-surface-300">{o.id.slice(-10)}</code>
                  </td>
                  <td className="px-5 py-3">
                    <p className="text-surface-200">{o.user.name || "—"}</p>
                    <p className="text-surface-500 text-xs">{o.user.email}</p>
                  </td>
                  <td className="px-5 py-3 text-surface-300 text-xs">
                    {o.items
                      .map(i =>
                        `${i.productName}${i.variantName ? ` (${i.variantName})` : ""}×${i.quantity}`
                      )
                      .join(", ")}
                  </td>
                  <td className="px-5 py-3 text-right font-mono">{formatKrw(o.totalKrw)}</td>
                  <td className="px-5 py-3 text-center">
                    <span className={`text-xs font-semibold px-2 py-0.5 rounded ${statusBadge(o.status)}`}>
                      {o.status}
                    </span>
                  </td>
                  <td className="px-5 py-3 text-center text-xs text-surface-400">
                    {o.shipment ? (
                      <span>
                        {o.shipment.courier}{" "}
                        <code className="text-surface-300">{o.shipment.trackingNumber}</code>
                      </span>
                    ) : (
                      <span className="text-surface-600">—</span>
                    )}
                  </td>
                  <td className="px-5 py-3 text-right">
                    <Link
                      href={`/admin/orders/${o.id}`}
                      className="text-xs text-indigo-300 hover:text-indigo-200 underline"
                    >
                      열기
                    </Link>
                  </td>
                </tr>
              ))}
              {orders.length === 0 && (
                <tr>
                  <td colSpan={8} className="px-5 py-8 text-center text-surface-500 text-sm">
                    아직 주문이 없습니다.
                  </td>
                </tr>
              )}
            </tbody>
          </table>
        </div>
      </div>
    </>
  );
}
