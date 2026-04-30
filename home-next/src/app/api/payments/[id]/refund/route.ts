import { NextResponse } from "next/server";
import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";
import { cancelPortonePayment } from "@/lib/portone";
import { applyNonPaidPayment } from "@/lib/payments";
import { getPortonePayment } from "@/lib/portone";

const REFUND_WINDOW_MS = 7 * 24 * 60 * 60 * 1000; // 전자상거래법 7일

export async function POST(
  req: Request,
  ctx: { params: Promise<{ id: string }> }
) {
  const session = await auth();
  if (!session?.user?.id) {
    return NextResponse.json({ error: "UNAUTHORIZED" }, { status: 401 });
  }
  const { id } = await ctx.params;

  const payment = await prisma.payment.findUnique({ where: { id } });
  if (!payment) {
    return NextResponse.json({ error: "NOT_FOUND" }, { status: 404 });
  }
  if (payment.userId !== session.user.id) {
    return NextResponse.json({ error: "FORBIDDEN" }, { status: 403 });
  }
  if (payment.status !== "PAID") {
    return NextResponse.json(
      { error: "NOT_REFUNDABLE", status: payment.status },
      { status: 409 }
    );
  }
  if (!payment.paidAt) {
    return NextResponse.json({ error: "MISSING_PAID_AT" }, { status: 409 });
  }

  const elapsed = Date.now() - payment.paidAt.getTime();
  if (elapsed > REFUND_WINDOW_MS) {
    return NextResponse.json(
      { error: "REFUND_WINDOW_EXPIRED", windowDays: 7 },
      { status: 409 }
    );
  }

  const body = (await req.json().catch(() => ({}))) as { reason?: string };
  const reason = (body.reason || "사용자 요청").slice(0, 200);

  try {
    await cancelPortonePayment({
      paymentId: payment.portonePaymentId,
      reason,
      requester: "CUSTOMER",
    });
  } catch (err) {
    console.error("[payments/refund] PortOne cancel failed", err);
    return NextResponse.json({ error: "PORTONE_CANCEL_FAILED" }, { status: 502 });
  }

  // Re-fetch authoritative state and apply (revokes entitlement).
  const fresh = await getPortonePayment(payment.portonePaymentId);
  await applyNonPaidPayment({
    portonePaymentId: payment.portonePaymentId,
    portonePayment: fresh,
  });

  return NextResponse.json({ ok: true });
}
