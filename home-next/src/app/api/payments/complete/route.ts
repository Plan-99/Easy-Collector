import { NextResponse } from "next/server";
import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";
import { getPortonePayment } from "@/lib/portone";
import { applyPaidPayment, applyNonPaidPayment } from "@/lib/payments";

// Trust-but-verify: client tells us the paymentId, we re-fetch from PortOne API
// and only mark PAID + grant entitlement if the server-side state agrees.
// Idempotent — webhook may also fire, both go through applyPaidPayment.
export async function POST(req: Request) {
  const session = await auth();
  if (!session?.user?.id) {
    return NextResponse.json({ error: "UNAUTHORIZED" }, { status: 401 });
  }

  const body = (await req.json().catch(() => ({}))) as { paymentId?: string };
  const paymentId = (body.paymentId || "").trim();
  if (!paymentId) {
    return NextResponse.json({ error: "PAYMENT_ID_REQUIRED" }, { status: 400 });
  }

  const localPayment = await prisma.payment.findUnique({
    where: { portonePaymentId: paymentId },
  });
  if (!localPayment) {
    return NextResponse.json({ error: "PAYMENT_NOT_FOUND" }, { status: 404 });
  }
  if (localPayment.userId !== session.user.id) {
    return NextResponse.json({ error: "FORBIDDEN" }, { status: 403 });
  }

  if (localPayment.status === "PAID") {
    return NextResponse.json({ status: "PAID", alreadyApplied: true });
  }

  let portonePayment;
  try {
    portonePayment = await getPortonePayment(paymentId);
  } catch (err) {
    console.error("[payments/complete] PortOne lookup failed", err);
    return NextResponse.json({ error: "PORTONE_LOOKUP_FAILED" }, { status: 502 });
  }

  if (portonePayment.status === "PAID") {
    await applyPaidPayment({ portonePaymentId: paymentId, portonePayment });
    return NextResponse.json({ status: "PAID", alreadyApplied: false });
  }

  await applyNonPaidPayment({ portonePaymentId: paymentId, portonePayment });
  return NextResponse.json(
    { error: "NOT_PAID", status: portonePayment.status },
    { status: 409 }
  );
}
