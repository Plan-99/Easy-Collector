import { NextResponse } from "next/server";
import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";
import { parseAddressBody } from "../route";

async function requireOwner(addrId: string) {
  const session = await auth();
  if (!session?.user?.id)
    return { error: NextResponse.json({ error: "UNAUTHORIZED" }, { status: 401 }) };
  const addr = await prisma.address.findUnique({ where: { id: addrId } });
  if (!addr) return { error: NextResponse.json({ error: "NOT_FOUND" }, { status: 404 }) };
  if (addr.userId !== session.user.id)
    return { error: NextResponse.json({ error: "FORBIDDEN" }, { status: 403 }) };
  return { userId: session.user.id, address: addr };
}

export async function PATCH(
  req: Request,
  ctx: { params: Promise<{ id: string }> }
) {
  const { id } = await ctx.params;
  const guard = await requireOwner(id);
  if (guard.error) return guard.error;

  const body = (await req.json().catch(() => ({}))) as Record<string, unknown>;
  const parsed = parseAddressBody(body);
  if ("error" in parsed) return parsed.error;

  const updated = await prisma.$transaction(async tx => {
    if (parsed.isDefault) {
      await tx.address.updateMany({
        where: { userId: guard.userId, isDefault: true, NOT: { id } },
        data: { isDefault: false },
      });
    }
    return tx.address.update({ where: { id }, data: parsed });
  });
  return NextResponse.json({ ok: true, address: updated });
}

export async function DELETE(
  _req: Request,
  ctx: { params: Promise<{ id: string }> }
) {
  const { id } = await ctx.params;
  const guard = await requireOwner(id);
  if (guard.error) return guard.error;
  await prisma.address.delete({ where: { id } });
  return NextResponse.json({ ok: true });
}
