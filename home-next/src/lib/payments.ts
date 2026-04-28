import "server-only";
import { prisma } from "@/lib/prisma";
import type { Payment as PortonePayment } from "@portone/server-sdk/payment";

// Apply a confirmed PortOne PAID result to our local Payment + grant
// Entitlement. Idempotent — safe to call from /complete and /webhook
// for the same paymentId.
export async function applyPaidPayment(opts: {
  portonePaymentId: string;
  portonePayment: PortonePayment;
}) {
  const { portonePaymentId, portonePayment } = opts;
  if (portonePayment.status !== "PAID") {
    throw new Error(`applyPaidPayment called on non-PAID status=${String(portonePayment.status)}`);
  }

  await prisma.$transaction(async tx => {
    const local = await tx.payment.findUnique({
      where: { portonePaymentId },
    });
    if (!local) {
      console.warn(`[applyPaidPayment] no local payment for ${portonePaymentId}`);
      return;
    }
    if (local.status === "PAID") return; // already applied

    if (portonePayment.amount.total !== local.amountKrw) {
      console.error(
        `[applyPaidPayment] amount mismatch ${portonePaymentId} expected=${local.amountKrw} actual=${portonePayment.amount.total}`
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

    if (local.moduleId) {
      await tx.entitlement.upsert({
        where: { userId_moduleId: { userId: local.userId, moduleId: local.moduleId } },
        update: { paymentId: local.id },
        create: { userId: local.userId, moduleId: local.moduleId, paymentId: local.id },
      });
    }
  });
}

// Reflect a non-PAID PortOne status on our local Payment (no entitlement
// changes here). Used by webhook for FAILED / CANCELLED / PARTIAL_CANCELLED.
export async function applyNonPaidPayment(opts: {
  portonePaymentId: string;
  portonePayment: PortonePayment;
}) {
  const { portonePaymentId, portonePayment } = opts;
  const status = mapPortoneStatus(portonePayment.status);
  if (!status) return;

  await prisma.$transaction(async tx => {
    const local = await tx.payment.findUnique({ where: { portonePaymentId } });
    if (!local) return;
    if (local.status === status) return;

    await tx.payment.update({
      where: { id: local.id },
      data: {
        status,
        cancelledAt:
          status === "CANCELLED" || status === "REFUNDED" ? new Date() : local.cancelledAt,
        raw: portonePayment as object,
      },
    });

    if (status === "REFUNDED" || status === "CANCELLED") {
      // Revoke entitlement granted by this payment.
      if (local.moduleId) {
        await tx.entitlement.deleteMany({
          where: {
            userId: local.userId,
            moduleId: local.moduleId,
            paymentId: local.id,
          },
        });
      }
    }
  });
}

function mapPortoneStatus(s: string | symbol): string | null {
  if (typeof s !== "string") return null;
  switch (s) {
    case "PAID":
      return "PAID";
    case "FAILED":
      return "FAILED";
    case "CANCELLED":
      return "CANCELLED";
    case "PARTIAL_CANCELLED":
      return "REFUNDED";
    case "PAY_PENDING":
    case "READY":
    case "VIRTUAL_ACCOUNT_ISSUED":
      return "PENDING";
    default:
      return null;
  }
}
