import { NextResponse } from "next/server";
import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";

// GET  list current user's saved addresses
// POST { label?, recipientName, phone, postalCode, addressLine1, addressLine2?,
//        requestNote?, isDefault? } — create. Sets isDefault=true atomically
//   if requested (clearing previous default).
export async function GET() {
  const session = await auth();
  if (!session?.user?.id)
    return NextResponse.json({ error: "UNAUTHORIZED" }, { status: 401 });
  const addresses = await prisma.address.findMany({
    where: { userId: session.user.id },
    orderBy: [{ isDefault: "desc" }, { createdAt: "desc" }],
  });
  return NextResponse.json({ addresses });
}

export async function POST(req: Request) {
  const session = await auth();
  if (!session?.user?.id)
    return NextResponse.json({ error: "UNAUTHORIZED" }, { status: 401 });
  const userId = session.user.id;

  const body = (await req.json().catch(() => ({}))) as Record<string, unknown>;
  const data = parseAddressBody(body);
  if ("error" in data) return data.error;

  // If isDefault is requested or this is the first saved address, ensure
  // exactly one default exists for this user.
  const existingCount = await prisma.address.count({ where: { userId } });
  const shouldBeDefault = data.isDefault || existingCount === 0;

  const created = await prisma.$transaction(async tx => {
    if (shouldBeDefault) {
      await tx.address.updateMany({
        where: { userId, isDefault: true },
        data: { isDefault: false },
      });
    }
    return tx.address.create({
      data: { ...data, isDefault: shouldBeDefault, userId },
    });
  });
  return NextResponse.json({ ok: true, address: created });
}

export function parseAddressBody(body: Record<string, unknown>):
  | {
      label: string | null;
      recipientName: string;
      phone: string;
      postalCode: string;
      addressLine1: string;
      addressLine2: string | null;
      requestNote: string | null;
      isDefault: boolean;
    }
  | { error: NextResponse } {
  const recipientName = String(body.recipientName || "").trim().slice(0, 100);
  const phone = String(body.phone || "").trim().slice(0, 30);
  const postalCode = String(body.postalCode || "").trim().slice(0, 10);
  const addressLine1 = String(body.addressLine1 || "").trim().slice(0, 200);
  if (!recipientName || !phone || !postalCode || !addressLine1) {
    return {
      error: NextResponse.json({ error: "MISSING_FIELDS" }, { status: 400 }),
    };
  }
  const addressLine2 = body.addressLine2
    ? String(body.addressLine2).trim().slice(0, 200) || null
    : null;
  const requestNote = body.requestNote
    ? String(body.requestNote).trim().slice(0, 500) || null
    : null;
  const label = body.label
    ? String(body.label).trim().slice(0, 50) || null
    : null;
  return {
    label,
    recipientName,
    phone,
    postalCode,
    addressLine1,
    addressLine2,
    requestNote,
    isDefault: body.isDefault === true,
  };
}
