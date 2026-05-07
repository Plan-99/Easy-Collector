import { NextResponse } from "next/server";
import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";
import { cancelPortonePayment, getPortonePayment } from "@/lib/portone";

async function requireAdmin() {
  const session = await auth();
  if (!session?.user?.id)
    return { error: NextResponse.json({ error: "UNAUTHORIZED" }, { status: 401 }) };
  const me = await prisma.user.findUnique({ where: { id: session.user.id } });
  if (me?.role !== "admin")
    return { error: NextResponse.json({ error: "FORBIDDEN" }, { status: 403 }) };
  return { adminId: session.user.id };
}

// Cancel a hardware order. Refunds the latest PAID payment via PortOne and
// flips Order.status to CANCELLED. Stock is restored to the product. If the
// order is already SHIPPED admin should still recover the package separately.
export async function POST(
  req: Request,
  ctx: { params: Promise<{ id: string }> }
) {
  const guard = await requireAdmin();
  if (guard.error) return guard.error;
  const { id } = await ctx.params;

  const order = await prisma.order.findUnique({
    where: { id },
    include: {
      items: true,
      payments: { orderBy: { createdAt: "desc" } },
    },
  });
  if (!order) return NextResponse.json({ error: "NOT_FOUND" }, { status: 404 });
  if (order.status === "CANCELLED" || order.status === "REFUNDED") {
    return NextResponse.json({ error: "ALREADY_CANCELLED" }, { status: 409 });
  }

  const body = (await req.json().catch(() => ({}))) as { reason?: string };
  const reason = (body.reason || "관리자 취소").slice(0, 200);
  const paid = order.payments.find(p => p.status === "PAID");

  if (paid) {
    try {
      await cancelPortonePayment({
        paymentId: paid.portonePaymentId,
        reason,
        requester: "ADMIN",
      });
      const fresh = await getPortonePayment(paid.portonePaymentId);
      const newStatus =
        fresh.status === "CANCELLED" || fresh.status === "PARTIAL_CANCELLED"
          ? "REFUNDED"
          : paid.status;
      await prisma.payment.update({
        where: { id: paid.id },
        data: {
          status: newStatus,
          cancelledAt: new Date(),
          raw: fresh as object,
        },
      });
    } catch (err) {
      console.error("[admin/orders/cancel] PortOne cancel failed", err);
      return NextResponse.json({ error: "PORTONE_CANCEL_FAILED" }, { status: 502 });
    }
  }

  // Restore stock + flip order status in a single transaction.
  // Increment from variant if the order item used one, otherwise the product.
  await prisma.$transaction([
    ...order.items.map(it =>
      it.productVariantId
        ? prisma.productVariant.update({
            where: { id: it.productVariantId },
            data: { stock: { increment: it.quantity } },
          })
        : prisma.product.update({
            where: { id: it.productId },
            data: { stock: { increment: it.quantity } },
          })
    ),
    prisma.order.update({
      where: { id },
      data: { status: paid ? "REFUNDED" : "CANCELLED" },
    }),
  ]);

  console.log(`[admin/orders/cancel] ${id} cancelled by ${guard.adminId}`);
  return NextResponse.json({ ok: true });
}
