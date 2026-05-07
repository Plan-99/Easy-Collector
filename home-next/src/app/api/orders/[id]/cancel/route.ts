import { NextResponse } from "next/server";
import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";
import { getPortonePayment } from "@/lib/portone";
import { applyPaidOrder } from "@/lib/orders";

// Cancel a PENDING hardware order from the customer side.
//
// Safety: race with PortOne — the user may click 취소 *after* paying but
// before /api/orders/complete had a chance to apply the result. So before
// flipping to CANCELLED we ask PortOne what the payment looks like; if it
// reports PAID we apply the paid result instead and refuse to cancel.
//
// PENDING orders never decremented stock (applyPaidOrder does that on
// payment confirmation), so we don't need to restore anything.
export async function POST(
  _req: Request,
  ctx: { params: Promise<{ id: string }> }
) {
  const session = await auth();
  if (!session?.user?.id)
    return NextResponse.json({ error: "UNAUTHORIZED" }, { status: 401 });
  const userId = session.user.id;
  const { id } = await ctx.params;

  const order = await prisma.order.findUnique({
    where: { id },
    include: { payments: { orderBy: { createdAt: "desc" } } },
  });
  if (!order || order.userId !== userId) {
    return NextResponse.json({ error: "NOT_FOUND" }, { status: 404 });
  }
  if (order.status !== "PENDING") {
    return NextResponse.json({ error: "NOT_PENDING" }, { status: 409 });
  }

  // Reconcile with PortOne first. Multiple Payment rows may exist if the user
  // retried; check every PENDING one.
  for (const p of order.payments) {
    if (p.status !== "PENDING") continue;
    try {
      const remote = await getPortonePayment(p.portonePaymentId);
      if (remote.status === "PAID") {
        await applyPaidOrder({
          portonePaymentId: p.portonePaymentId,
          portonePayment: remote,
        });
        return NextResponse.json({ error: "ALREADY_PAID" }, { status: 409 });
      }
    } catch (err) {
      // PortOne returns 404 for paymentIds that were never attempted — that's
      // expected when the user closed the SDK before submitting. Swallow and
      // continue with cancellation.
      console.warn(`[orders/cancel] PortOne lookup failed for ${p.portonePaymentId}`, err);
    }
  }

  await prisma.$transaction([
    prisma.payment.updateMany({
      where: { orderId: order.id, status: "PENDING" },
      data: { status: "CANCELLED", cancelledAt: new Date() },
    }),
    prisma.order.update({
      where: { id: order.id },
      data: { status: "CANCELLED" },
    }),
  ]);

  return NextResponse.json({ ok: true });
}
