import { prisma } from "@/lib/prisma";
import { notFound } from "next/navigation";
import Link from "next/link";
import ShipmentForm from "./ShipmentForm";
import CancelOrderButton from "./CancelOrderButton";

function formatKrw(amountKrw: number) {
  return new Intl.NumberFormat("ko-KR", {
    style: "currency",
    currency: "KRW",
    maximumFractionDigits: 0,
  }).format(amountKrw);
}

export default async function AdminOrderDetailPage({
  params,
}: {
  params: Promise<{ id: string }>;
}) {
  const { id } = await params;
  const o = await prisma.order.findUnique({
    where: { id },
    include: {
      user: { select: { email: true, name: true } },
      items: { include: { product: true } },
      shipment: true,
      payments: { orderBy: { createdAt: "desc" } },
    },
  });
  if (!o) notFound();

  const paidPayment = o.payments.find(p => p.status === "PAID");

  return (
    <>
      <Link
        href="/admin/orders"
        className="text-xs text-surface-400 hover:text-white underline"
      >
        ← 주문 목록
      </Link>

      <div className="grid md:grid-cols-2 gap-4 mt-4">
        <div className="bezel-card">
          <div className="bezel-inner p-6 space-y-3">
            <h3 className="font-bold text-surface-100">주문 정보</h3>
            <Row label="주문번호" value={<code className="text-xs">{o.id}</code>} />
            <Row label="상태" value={o.status} />
            <Row label="주문일시" value={new Date(o.createdAt).toLocaleString("ko-KR")} />
            <Row label="총 금액" value={<span className="font-mono">{formatKrw(o.totalKrw)}</span>} />
            <Row
              label="결제수단"
              value={paidPayment ? `${paidPayment.method || "—"} (${paidPayment.pgProvider || "—"})` : "—"}
            />
            {paidPayment?.receiptUrl && (
              <Row
                label="영수증"
                value={
                  <a
                    href={paidPayment.receiptUrl}
                    target="_blank"
                    className="text-indigo-300 underline text-xs"
                    rel="noreferrer"
                  >
                    열기
                  </a>
                }
              />
            )}
          </div>
        </div>

        <div className="bezel-card">
          <div className="bezel-inner p-6 space-y-3">
            <h3 className="font-bold text-surface-100">주문자 / 배송지</h3>
            <Row label="이메일" value={o.user.email} />
            <Row label="회원명" value={o.user.name || "—"} />
            <Row label="받는 사람" value={o.recipientName} />
            <Row label="연락처" value={o.phone} />
            <Row
              label="주소"
              value={
                <span className="text-sm">
                  ({o.postalCode}) {o.addressLine1}
                  {o.addressLine2 ? ` ${o.addressLine2}` : ""}
                </span>
              }
            />
            {o.requestNote && <Row label="요청사항" value={o.requestNote} />}
          </div>
        </div>
      </div>

      <div className="bezel-card mt-4">
        <div className="bezel-inner p-6">
          <h3 className="font-bold text-surface-100 mb-3">상품</h3>
          <table className="w-full text-sm">
            <thead>
              <tr className="border-b border-white/5 text-left text-surface-500 text-xs">
                <th className="py-2">상품</th>
                <th className="py-2">SKU</th>
                <th className="py-2 text-right">단가</th>
                <th className="py-2 text-right">수량</th>
                <th className="py-2 text-right">합계</th>
              </tr>
            </thead>
            <tbody>
              {o.items.map(i => (
                <tr key={i.id} className="border-b border-white/5">
                  <td className="py-2.5">
                    {i.productName}
                    {i.variantName && (
                      <span className="text-xs text-indigo-300 ml-2">({i.variantName})</span>
                    )}
                  </td>
                  <td className="py-2.5 font-mono text-xs text-surface-400">
                    {i.variantSku || i.productSku}
                  </td>
                  <td className="py-2.5 text-right font-mono">{formatKrw(i.unitPriceKrw)}</td>
                  <td className="py-2.5 text-right">{i.quantity}</td>
                  <td className="py-2.5 text-right font-mono">
                    {formatKrw(i.unitPriceKrw * i.quantity)}
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>

      <div className="bezel-card mt-4">
        <div className="bezel-inner p-6">
          <h3 className="font-bold text-surface-100 mb-3">배송 / 송장</h3>
          <ShipmentForm
            orderId={o.id}
            initial={
              o.shipment
                ? {
                    courier: o.shipment.courier,
                    trackingNumber: o.shipment.trackingNumber,
                    shippedAt: o.shipment.shippedAt
                      ? o.shipment.shippedAt.toISOString().slice(0, 10)
                      : "",
                  }
                : null
            }
            orderStatus={o.status}
          />
        </div>
      </div>

      {(o.status === "PAID" || o.status === "SHIPPED") && paidPayment && (
        <div className="bezel-card mt-4">
          <div className="bezel-inner p-6 space-y-3">
            <h3 className="font-bold text-red-300">주문 취소 / 환불</h3>
            <p className="text-sm text-surface-400">
              주문을 취소하면 PortOne을 통해 결제도 즉시 환불 처리됩니다. 이미 출고된 상품은
              회수 절차를 별도로 진행해 주세요.
            </p>
            <CancelOrderButton orderId={o.id} />
          </div>
        </div>
      )}
    </>
  );
}

function Row({ label, value }: { label: string; value: React.ReactNode }) {
  return (
    <div className="flex items-baseline gap-3 text-sm">
      <span className="w-24 shrink-0 text-surface-500 text-xs">{label}</span>
      <span className="text-surface-200">{value}</span>
    </div>
  );
}
