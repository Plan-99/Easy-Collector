import { NextRequest } from "next/server";
import { prisma } from "@/lib/prisma";
import { authenticateLauncher, unauthorized } from "@/lib/launcher-auth";

// Returns module IDs the authenticated user owns. Launcher gates installs on this.
// Free modules (priceKrw=0) are implicitly entitled — no row needed.
export async function GET(req: NextRequest) {
  const auth = await authenticateLauncher(req);
  if (!auth) return unauthorized();

  const [paid, freeModules] = await Promise.all([
    prisma.entitlement.findMany({
      where: { userId: auth.user.id },
      select: { moduleId: true, grantedAt: true, paymentId: true },
    }),
    prisma.module.findMany({
      where: { active: true, priceKrw: 0 },
      select: { id: true },
    }),
  ]);

  const owned = new Set<string>();
  for (const e of paid) owned.add(e.moduleId);
  for (const m of freeModules) owned.add(m.id);

  return Response.json({
    moduleIds: Array.from(owned).sort(),
    paid: paid.map(e => ({
      moduleId: e.moduleId,
      grantedAt: e.grantedAt.toISOString(),
      paid: e.paymentId !== null,
    })),
  });
}
