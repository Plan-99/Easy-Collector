import { NextResponse } from "next/server";
import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";

async function requireAdmin() {
  const session = await auth();
  if (!session?.user?.id)
    return { error: NextResponse.json({ error: "UNAUTHORIZED" }, { status: 401 }) };
  const me = await prisma.user.findUnique({ where: { id: session.user.id } });
  if (me?.role !== "admin")
    return { error: NextResponse.json({ error: "FORBIDDEN" }, { status: 403 }) };
  return { adminId: session.user.id };
}

// Two operations on the same endpoint:
//   - register/update tracking: { courier, trackingNumber, shippedAt? }
//     → upserts Shipment + Order.status = SHIPPED
//   - mark delivered: { markDelivered: true }
//     → Shipment.deliveredAt = now + Order.status = DELIVERED
export async function POST(
  req: Request,
  ctx: { params: Promise<{ id: string }> }
) {
  const guard = await requireAdmin();
  if (guard.error) return guard.error;
  const { id } = await ctx.params;

  const order = await prisma.order.findUnique({ where: { id } });
  if (!order) return NextResponse.json({ error: "NOT_FOUND" }, { status: 404 });

  const body = (await req.json().catch(() => ({}))) as {
    courier?: string;
    trackingNumber?: string;
    shippedAt?: string | null;
    markDelivered?: boolean;
  };

  if (body.markDelivered) {
    if (order.status !== "SHIPPED") {
      return NextResponse.json(
        { error: "NOT_SHIPPED", currentStatus: order.status },
        { status: 409 }
      );
    }
    await prisma.$transaction([
      prisma.shipment.update({
        where: { orderId: id },
        data: { deliveredAt: new Date() },
      }),
      prisma.order.update({ where: { id }, data: { status: "DELIVERED" } }),
    ]);
    return NextResponse.json({ ok: true, status: "DELIVERED" });
  }

  const courier = (body.courier || "").trim();
  const trackingNumber = (body.trackingNumber || "").trim();
  if (!courier || !trackingNumber) {
    return NextResponse.json({ error: "MISSING_FIELDS" }, { status: 400 });
  }
  if (order.status !== "PAID" && order.status !== "SHIPPED") {
    return NextResponse.json(
      { error: "NOT_PAID", currentStatus: order.status },
      { status: 409 }
    );
  }

  const shippedAt = body.shippedAt ? new Date(body.shippedAt + "T00:00:00") : new Date();

  await prisma.$transaction([
    prisma.shipment.upsert({
      where: { orderId: id },
      update: { courier, trackingNumber, shippedAt },
      create: { orderId: id, courier, trackingNumber, shippedAt },
    }),
    prisma.order.update({ where: { id }, data: { status: "SHIPPED" } }),
  ]);

  console.log(`[admin/orders] ${id} shipment registered by ${guard.adminId}`);
  return NextResponse.json({ ok: true, status: "SHIPPED" });
}
