import { NextResponse } from "next/server";
import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";
import { buildPaymentId } from "@/lib/portone";

export async function POST(req: Request) {
  const session = await auth();
  if (!session?.user?.id) {
    return NextResponse.json({ error: "UNAUTHORIZED" }, { status: 401 });
  }

  const body = (await req.json().catch(() => ({}))) as { moduleId?: string };
  const moduleId = (body.moduleId || "").trim();
  if (!moduleId) {
    return NextResponse.json({ error: "MODULE_ID_REQUIRED" }, { status: 400 });
  }

  const moduleRow = await prisma.module.findUnique({ where: { id: moduleId } });
  if (!moduleRow || !moduleRow.active) {
    return NextResponse.json({ error: "MODULE_NOT_FOUND" }, { status: 404 });
  }
  if (moduleRow.priceKrw <= 0) {
    return NextResponse.json({ error: "MODULE_IS_FREE" }, { status: 400 });
  }

  const owned = await prisma.entitlement.findUnique({
    where: { userId_moduleId: { userId: session.user.id, moduleId } },
  });
  if (owned) {
    return NextResponse.json({ error: "ALREADY_OWNED" }, { status: 409 });
  }

  const paymentId = buildPaymentId();
  const orderName =
    moduleRow.name.length > 60 ? moduleRow.name.slice(0, 57) + "…" : moduleRow.name;

  await prisma.payment.create({
    data: {
      userId: session.user.id,
      moduleId,
      portonePaymentId: paymentId,
      status: "PENDING",
      amountKrw: moduleRow.priceKrw,
    },
  });

  return NextResponse.json({
    paymentId,
    amountKrw: moduleRow.priceKrw,
    orderName,
  });
}
