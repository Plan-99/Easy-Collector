import "server-only";
import { prisma } from "@/lib/prisma";
import type { Payment as PortonePayment } from "@portone/server-sdk/payment";

// Apply a confirmed PortOne PAID result to an HW Order. Decrements stock,
// flips Order.status to PAID, updates the linked Payment row. Idempotent —
// safe to call from /api/orders/complete and the shared webhook.
export async function applyPaidOrder(opts: {
  portonePaymentId: string;
  portonePayment: PortonePayment;
}) {
  const { portonePaymentId, portonePayment } = opts;
  if (portonePayment.status !== "PAID") {
    throw new Error(
      `applyPaidOrder called on non-PAID status=${String(portonePayment.status)}`
    );
  }

  await prisma.$transaction(async tx => {
    const local = await tx.payment.findUnique({ where: { portonePaymentId } });
    if (!local || !local.orderId) return; // not an order payment

    const order = await tx.order.findUnique({
      where: { id: local.orderId },
      include: { items: true },
    });
    if (!order) return;
    if (order.status !== "PENDING" && local.status === "PAID") return;

    if (portonePayment.amount.total !== local.amountKrw) {
      console.error(
        `[applyPaidOrder] amount mismatch ${portonePaymentId} expected=${local.amountKrw} actual=${portonePayment.amount.total}`
      );
      return;
    }

    const rawMethodType = portonePayment.method?.type;
    const methodTypeStr = typeof rawMethodType === "string" ? rawMethodType : null;
    const method =
      methodTypeStr === "PaymentMethodCard"
        ? "CARD"
        : methodTypeStr === "PaymentMethodEasyPay"
          ? "EASY_PAY"
          : methodTypeStr;

    await tx.payment.update({
      where: { id: local.id },
      data: {
        status: "PAID",
        method,
        pgProvider: portonePayment.channel?.pgProvider || null,
        receiptUrl: portonePayment.receiptUrl || null,
        paidAt: portonePayment.paidAt ? new Date(portonePayment.paidAt) : new Date(),
        raw: portonePayment as object,
      },
    });

    if (order.status === "PENDING") {
      // Decrement stock per item — from variant if specified, else product.
      for (const item of order.items) {
        if (item.productVariantId) {
          await tx.productVariant.update({
            where: { id: item.productVariantId },
            data: { stock: { decrement: item.quantity } },
          });
        } else {
          await tx.product.update({
            where: { id: item.productId },
            data: { stock: { decrement: item.quantity } },
          });
        }
      }
      await tx.order.update({ where: { id: order.id }, data: { status: "PAID" } });
    }
  });
}
