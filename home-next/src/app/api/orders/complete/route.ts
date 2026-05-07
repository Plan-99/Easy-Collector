import { NextResponse } from "next/server";
import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";
import { getPortonePayment } from "@/lib/portone";
import { applyPaidOrder } from "@/lib/orders";
import { applyNonPaidPayment } from "@/lib/payments";

// Trust-but-verify the order payment after the PortOne SDK promise resolves.
// Same idempotent shape as /api/payments/complete but for HW orders.
export async function POST(req: Request) {
  const session = await auth();
  if (!session?.user?.id)
    return NextResponse.json({ error: "UNAUTHORIZED" }, { status: 401 });

  const body = (await req.json().catch(() => ({}))) as { paymentId?: string };
  const paymentId = (body.paymentId || "").trim();
  if (!paymentId) return NextResponse.json({ error: "PAYMENT_ID_REQUIRED" }, { status: 400 });

  const local = await prisma.payment.findUnique({
    where: { portonePaymentId: paymentId },
    include: { order: true },
  });
  if (!local || !local.orderId)
    return NextResponse.json({ error: "ORDER_PAYMENT_NOT_FOUND" }, { status: 404 });
  if (local.userId !== session.user.id)
    return NextResponse.json({ error: "FORBIDDEN" }, { status: 403 });

  if (local.status === "PAID")
    return NextResponse.json({ status: "PAID", orderId: local.orderId, alreadyApplied: true });

  let portonePayment;
  try {
    portonePayment = await getPortonePayment(paymentId);
  } catch (err) {
    console.error("[orders/complete] PortOne lookup failed", err);
    return NextResponse.json({ error: "PORTONE_LOOKUP_FAILED" }, { status: 502 });
  }

  if (portonePayment.status === "PAID") {
    await applyPaidOrder({ portonePaymentId: paymentId, portonePayment });
    return NextResponse.json({ status: "PAID", orderId: local.orderId, alreadyApplied: false });
  }

  await applyNonPaidPayment({ portonePaymentId: paymentId, portonePayment });
  return NextResponse.json(
    { error: "NOT_PAID", status: portonePayment.status },
    { status: 409 }
  );
}
