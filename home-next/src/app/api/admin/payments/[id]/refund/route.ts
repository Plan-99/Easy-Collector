import { NextResponse } from "next/server";
import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";
import { cancelPortonePayment, getPortonePayment } from "@/lib/portone";
import { applyNonPaidPayment } from "@/lib/payments";

// Admin-initiated refund. Bypasses the 7-day customer window — admin
// discretion (e.g. customer service refund). Same idempotent flow as user
// refund: PortOne cancel → re-fetch → apply → revoke entitlement.
export async function POST(
  req: Request,
  ctx: { params: Promise<{ id: string }> }
) {
  const session = await auth();
  if (!session?.user?.id) {
    return NextResponse.json({ error: "UNAUTHORIZED" }, { status: 401 });
  }
  const me = await prisma.user.findUnique({ where: { id: session.user.id } });
  if (me?.role !== "admin") {
    return NextResponse.json({ error: "FORBIDDEN" }, { status: 403 });
  }

  const { id } = await ctx.params;
  const payment = await prisma.payment.findUnique({ where: { id } });
  if (!payment) return NextResponse.json({ error: "NOT_FOUND" }, { status: 404 });
  if (payment.status !== "PAID") {
    return NextResponse.json(
      { error: "NOT_REFUNDABLE", status: payment.status },
      { status: 409 }
    );
  }

  const body = (await req.json().catch(() => ({}))) as { reason?: string };
  const reason = (body.reason || "관리자 환불").slice(0, 200);

  try {
    await cancelPortonePayment({
      paymentId: payment.portonePaymentId,
      reason,
      requester: "ADMIN",
    });
  } catch (err) {
    console.error("[admin/refund] PortOne cancel failed", err);
    return NextResponse.json({ error: "PORTONE_CANCEL_FAILED" }, { status: 502 });
  }

  const fresh = await getPortonePayment(payment.portonePaymentId);
  await applyNonPaidPayment({
    portonePaymentId: payment.portonePaymentId,
    portonePayment: fresh,
  });
  console.log(`[admin/refund] payment=${payment.portonePaymentId} by adminId=${session.user.id}`);

  return NextResponse.json({ ok: true });
}
