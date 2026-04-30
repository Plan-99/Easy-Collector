import { prisma } from "@/lib/prisma";

// Public catalog used by the launcher to render install buttons + populate
// MODULE_REGISTRY at startup. No auth required — prices are not sensitive
// and the launcher must read this before sign-in to render preview UI.
export async function GET() {
  const modules = await prisma.module.findMany({
    where: { active: true },
    orderBy: [{ category: "asc" }, { id: "asc" }],
    select: {
      id: true,
      name: true,
      category: true,
      description: true,
      priceKrw: true,
      required: true,
      installByDefault: true,
      assetName: true,
      dependencies: true,
    },
  });
  return Response.json({ modules });
}
