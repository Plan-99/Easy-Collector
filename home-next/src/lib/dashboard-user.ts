import "server-only";
import { cache } from "react";
import { prisma } from "@/lib/prisma";

// Single source of user data for the dashboard. Wrapped with React cache()
// so a layout call and a page call within the same request hit the DB only
// once. Cuts Sydney-region Neon latency in half on every navigation.
export const getDashboardUser = cache(async (userId: string) => {
  return prisma.user.findUnique({
    where: { id: userId },
    include: {
      devices: { orderBy: { lastSeenAt: "desc" } },
      entitlements: {
        include: { module: true },
        orderBy: { grantedAt: "desc" },
      },
      payments: { orderBy: { createdAt: "desc" }, take: 20 },
      billingKeys: { where: { active: true }, orderBy: { createdAt: "desc" } },
    },
  });
});

export function isOnboarded(u: {
  organization: string | null;
  department: string | null;
  jobRole: string | null;
  termsAcceptedAt: Date | null;
  privacyAcceptedAt: Date | null;
  refundAcceptedAt: Date | null;
} | null): boolean {
  return (
    !!u?.organization &&
    !!u.department &&
    !!u.jobRole &&
    !!u.termsAcceptedAt &&
    !!u.privacyAcceptedAt &&
    !!u.refundAcceptedAt
  );
}
