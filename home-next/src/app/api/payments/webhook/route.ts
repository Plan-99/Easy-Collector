import { NextResponse } from "next/server";
import { prisma } from "@/lib/prisma";
import { getPortonePayment, verifyPortoneWebhook } from "@/lib/portone";
import { applyPaidPayment, applyNonPaidPayment } from "@/lib/payments";
import { applyPaidOrder } from "@/lib/orders";

// PortOne v2 async webhook receiver.
// - Verifies signature with PORTONE_WEBHOOK_SECRET
// - Re-fetches the payment from REST (do not trust the webhook body alone)
// - Applies the same idempotent logic as /api/payments/complete
//
// Configure in PortOne 콘솔 → 결제 연동 → 웹훅에 다음 URL 등록:
//   https://easytrainerhome.vercel.app/api/payments/webhook
export async function POST(req: Request) {
  const rawBody = await req.text();
  const headers: Record<string, string> = {};
  req.headers.forEach((v, k) => {
    headers[k] = v;
  });

  let webhook;
  try {
    webhook = await verifyPortoneWebhook(rawBody, headers);
  } catch (err) {
    console.error("[payments/webhook] verification failed", err);
    return NextResponse.json({ error: "WEBHOOK_VERIFICATION_FAILED" }, { status: 400 });
  }

  // We only care about transaction-state webhooks here. Others
  // (billing-key, virtual-account, etc.) are acknowledged but ignored.
  const data = (webhook as { data?: { paymentId?: string } }).data;
  const portonePaymentId = data?.paymentId;
  if (!portonePaymentId) {
    return NextResponse.json({ ok: true, ignored: true });
  }

  let portonePayment;
  try {
    portonePayment = await getPortonePayment(portonePaymentId);
  } catch (err) {
    console.error("[payments/webhook] PortOne lookup failed", err);
    // 5xx so PortOne retries.
    return NextResponse.json({ error: "PORTONE_LOOKUP_FAILED" }, { status: 502 });
  }

  // Determine whether this is a Module payment or HW Order payment by
  // checking which FK is set on our local Payment row.
  const local = await prisma.payment.findUnique({
    where: { portonePaymentId },
    select: { moduleId: true, orderId: true },
  });

  if (portonePayment.status === "PAID") {
    if (local?.orderId) {
      await applyPaidOrder({ portonePaymentId, portonePayment });
    } else {
      await applyPaidPayment({ portonePaymentId, portonePayment });
    }
  } else {
    await applyNonPaidPayment({ portonePaymentId, portonePayment });
  }

  return NextResponse.json({ ok: true });
}
