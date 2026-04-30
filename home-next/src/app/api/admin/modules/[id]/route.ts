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

// Patch the module catalog row. Admin-only — used by /admin/modules edit UI.
export async function PATCH(
  req: Request,
  ctx: { params: Promise<{ id: string }> }
) {
  const guard = await requireAdmin();
  if (guard.error) return guard.error;

  const { id } = await ctx.params;
  const body = (await req.json().catch(() => ({}))) as Partial<{
    name: string;
    description: string;
    priceKrw: number;
    active: boolean;
  }>;

  const data: Record<string, unknown> = {};
  if (typeof body.name === "string") {
    const name = body.name.trim().slice(0, 200);
    if (!name) return NextResponse.json({ error: "NAME_REQUIRED" }, { status: 400 });
    data.name = name;
  }
  if (typeof body.description === "string") {
    data.description = body.description.trim().slice(0, 1000) || null;
  }
  if (typeof body.priceKrw === "number") {
    if (!Number.isFinite(body.priceKrw) || body.priceKrw < 0) {
      return NextResponse.json({ error: "INVALID_PRICE" }, { status: 400 });
    }
    data.priceKrw = Math.floor(body.priceKrw);
  }
  if (typeof body.active === "boolean") {
    data.active = body.active;
  }

  if (Object.keys(data).length === 0) {
    return NextResponse.json({ error: "NO_FIELDS" }, { status: 400 });
  }

  try {
    const updated = await prisma.module.update({ where: { id }, data });
    console.log(
      `[admin/modules] ${id} updated by ${guard.adminId}: ${JSON.stringify(data)}`
    );
    return NextResponse.json({ ok: true, module: updated });
  } catch (err: unknown) {
    if ((err as { code?: string })?.code === "P2025") {
      return NextResponse.json({ error: "MODULE_NOT_FOUND" }, { status: 404 });
    }
    console.error("[admin/modules] update failed", err);
    return NextResponse.json({ error: "UPDATE_FAILED" }, { status: 500 });
  }
}
