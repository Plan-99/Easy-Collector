import { NextResponse } from "next/server";
import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";

async function requireAdmin() {
  const session = await auth();
  if (!session?.user?.id) return { error: NextResponse.json({ error: "UNAUTHORIZED" }, { status: 401 }) };
  const me = await prisma.user.findUnique({ where: { id: session.user.id } });
  if (me?.role !== "admin") {
    return { error: NextResponse.json({ error: "FORBIDDEN" }, { status: 403 }) };
  }
  return { adminId: session.user.id };
}

// Grant a module to a user (admin override / manual issuance).
// POST { userId, moduleId }
export async function POST(req: Request) {
  const guard = await requireAdmin();
  if (guard.error) return guard.error;

  const body = (await req.json().catch(() => ({}))) as {
    userId?: string;
    moduleId?: string;
  };
  const userId = (body.userId || "").trim();
  const moduleId = (body.moduleId || "").trim();
  if (!userId || !moduleId) {
    return NextResponse.json({ error: "MISSING_FIELDS" }, { status: 400 });
  }

  const [user, mod] = await Promise.all([
    prisma.user.findUnique({ where: { id: userId } }),
    prisma.module.findUnique({ where: { id: moduleId } }),
  ]);
  if (!user) return NextResponse.json({ error: "USER_NOT_FOUND" }, { status: 404 });
  if (!mod) return NextResponse.json({ error: "MODULE_NOT_FOUND" }, { status: 404 });

  const ent = await prisma.entitlement.upsert({
    where: { userId_moduleId: { userId, moduleId } },
    update: {}, // already owned, nothing to do
    create: { userId, moduleId, paymentId: null }, // null = admin-granted / free
  });
  console.log(`[admin/entitlements] granted ${moduleId} to ${userId} by ${guard.adminId}`);

  return NextResponse.json({ ok: true, entitlementId: ent.id });
}

// Revoke (delete) a single entitlement.
// DELETE ?userId=...&moduleId=...
export async function DELETE(req: Request) {
  const guard = await requireAdmin();
  if (guard.error) return guard.error;

  const url = new URL(req.url);
  const userId = (url.searchParams.get("userId") || "").trim();
  const moduleId = (url.searchParams.get("moduleId") || "").trim();
  if (!userId || !moduleId) {
    return NextResponse.json({ error: "MISSING_FIELDS" }, { status: 400 });
  }

  const result = await prisma.entitlement.deleteMany({ where: { userId, moduleId } });
  console.log(`[admin/entitlements] revoked ${moduleId} from ${userId} by ${guard.adminId}`);
  return NextResponse.json({ ok: true, deleted: result.count });
}
