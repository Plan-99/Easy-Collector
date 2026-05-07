import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";
import { notFound, redirect } from "next/navigation";
import Link from "next/link";
import OrderRedirectVerify from "./OrderRedirectVerify";
import CancelPendingButton from "../CancelPendingButton";

const STATUS_LABEL: Record<string, string> = {
  PENDING: "결제 대기",
  PAID: "결제 완료 / 출고 준비",
  SHIPPED: "배송 중",
  DELIVERED: "배송 완료",
  CANCELLED: "취소",
  REFUNDED: "환불",
};

const COURIER_TRACKING_URL: Record<string, (n: string) => string> = {
  CJ대한통운: n => `https://trace.cjlogistics.com/web/detail.jsp?slipno=${n}`,
  한진택배: n => `https://www.hanjin.com/kor/CMS/DeliveryMgr/WaybillResult.do?wblnumText2=${n}`,
  롯데택배: n => `https://www.lotteglogis.com/home/reservation/tracking/index?invno=${n}`,
  우체국택배: n => `https://service.epost.go.kr/trace.RetrieveDomRigiTraceList.comm?sid1=${n}`,
  로젠택배: n => `https://www.ilogen.com/web/personal/trace/${n}`,
};

function formatKrw(amountKrw: number) {
  return new Intl.NumberFormat("ko-KR", {
    style: "currency",
    currency: "KRW",
    maximumFractionDigits: 0,
  }).format(amountKrw);
}

export default async function DashboardOrderDetailPage({
  params,
}: {
  params: Promise<{ id: string }>;
}) {
  const session = await auth();
  if (!session?.user?.id) redirect("/auth/signin");

  const { id } = await params;
  const o = await prisma.order.findUnique({
    where: { id },
    include: {
      items: { include: { product: { select: { imageUrl: true } } } },
      shipment: true,
      payments: { orderBy: { createdAt: "desc" } },
    },
  });
  if (!o) notFound();
  if (o.userId !== session.user.id) notFound();

  const trackUrl = o.shipment
    ? COURIER_TRACKING_URL[o.shipment.courier]?.(o.shipment.trackingNumber)
    : null;

  return (
    <>
      <OrderRedirectVerify />
      <Link
        href="/dashboard/orders"
        className="text-xs text-surface-400 hover:text-white underline"
      >
        ← 주문 목록
      </Link>

      <div className="bezel-card mt-4">
        <div className="bezel-inner p-6 space-y-3">
          <div className="flex items-center justify-between">
            <h2 className="font-bold text-lg">주문 상태</h2>
            <span className="text-xs font-semibold px-2.5 py-1 rounded bg-indigo-500/15 text-indigo-300">
              {STATUS_LABEL[o.status] || o.status}
            </span>
          </div>
          <p className="text-xs text-surface-500">
            주문일시: {new Date(o.createdAt).toLocaleString("ko-KR")} · 주문번호{" "}
            <code className="text-surface-300">{o.id.slice(-10)}</code>
          </p>
          {o.status === "PENDING" && (
            <div className="flex items-center justify-between gap-3 rounded-lg bg-amber-500/10 border border-amber-500/20 px-4 py-3 mt-3">
              <p className="text-xs text-amber-200">
                결제가 완료되지 않은 주문입니다. 결제하지 않으셨다면 취소할 수 있습니다.
              </p>
              <CancelPendingButton orderId={o.id} />
            </div>
          )}
          {o.shipment && (
            <div className="rounded-lg bg-white/5 border border-white/10 p-4 mt-3">
              <p className="text-sm font-semibold text-surface-100">
                {o.shipment.courier}{" "}
                <code className="font-mono text-surface-300">
                  {o.shipment.trackingNumber}
                </code>
              </p>
              {trackUrl && (
                <a
                  href={trackUrl}
                  target="_blank"
                  rel="noreferrer"
                  className="text-xs text-indigo-300 hover:text-indigo-200 underline"
                >
                  배송 추적하기 →
                </a>
              )}
              {o.shipment.shippedAt && (
                <p className="text-xs text-surface-500 mt-2">
                  출고일 {new Date(o.shipment.shippedAt).toLocaleDateString("ko-KR")}
                </p>
              )}
            </div>
          )}
        </div>
      </div>

      <div className="bezel-card mt-4">
        <div className="bezel-inner p-6">
          <h3 className="font-bold mb-3">상품</h3>
          <ul className="divide-y divide-white/5">
            {o.items.map(i => (
              <li key={i.id} className="py-3 flex items-center gap-4">
                {i.product?.imageUrl ? (
                  // eslint-disable-next-line @next/next/no-img-element
                  <img
                    src={i.product.imageUrl}
                    alt={i.productName}
                    className="w-14 h-14 rounded bg-white/5 border border-white/10 object-contain p-1.5"
                  />
                ) : (
                  <div className="w-14 h-14 rounded bg-white/5 border border-white/10" />
                )}
                <div className="flex-1 min-w-0">
                  <p className="font-medium text-surface-100">
                    {i.productName}
                    {i.variantName && (
                      <span className="text-xs text-indigo-300 ml-2">({i.variantName})</span>
                    )}
                  </p>
                  <p className="text-surface-500 text-xs">{i.variantSku || i.productSku}</p>
                </div>
                <div className="text-right text-sm">
                  <p className="font-mono">{formatKrw(i.unitPriceKrw)}</p>
                  <p className="text-surface-500 text-xs">× {i.quantity}</p>
                </div>
              </li>
            ))}
          </ul>
          <div className="flex items-center justify-between pt-3 mt-3 border-t border-white/10">
            <span className="text-sm text-surface-300">총 결제 금액</span>
            <span className="font-mono font-bold text-lg">{formatKrw(o.totalKrw)}</span>
          </div>
        </div>
      </div>

      <div className="bezel-card mt-4">
        <div className="bezel-inner p-6 space-y-2 text-sm">
          <h3 className="font-bold mb-2">배송지</h3>
          <p className="text-surface-200">
            {o.recipientName} · {o.phone}
          </p>
          <p className="text-surface-300">
            ({o.postalCode}) {o.addressLine1}
            {o.addressLine2 ? ` ${o.addressLine2}` : ""}
          </p>
          {o.requestNote && (
            <p className="text-surface-500 text-xs mt-1">요청사항: {o.requestNote}</p>
          )}
        </div>
      </div>
    </>
  );
}
