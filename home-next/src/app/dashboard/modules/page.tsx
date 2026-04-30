import { auth, signOut } from "@/auth";
import { prisma } from "@/lib/prisma";
import { redirect } from "next/navigation";
import ModuleCatalog from "./ModuleCatalog";

const CATEGORY_LABELS: Record<string, string> = {
  robot: "로봇",
  sensor: "센서",
  extension: "확장 기능",
};
const VISIBLE_CATEGORIES = ["robot", "sensor", "extension"];

export default async function DashboardModulesPage({
  searchParams,
}: {
  searchParams: Promise<{ focus?: string; u?: string }>;
}) {
  const session = await auth();
  if (!session?.user?.id) redirect("/auth/signin");

  const sp = await searchParams;
  const focusedId = typeof sp.focus === "string" ? sp.focus.trim() : "";
  const expectedUserId = typeof sp.u === "string" ? sp.u.trim() : "";

  if (expectedUserId && expectedUserId !== session.user.id) {
    return (
      <div className="bezel-card">
        <div className="bezel-inner p-8 space-y-4">
          <h2 className="font-bold text-lg text-red-300">
            다른 계정으로 로그인되어 있습니다
          </h2>
          <p className="text-sm text-surface-300">
            런처에 등록된 계정과 다른 계정({session.user.email})으로 브라우저에 로그인되어
            있어 결제 흐름을 진행할 수 없습니다. 로그아웃 후 런처와 동일한 계정으로
            다시 로그인해 주세요.
          </p>
          <form
            action={async () => {
              "use server";
              await signOut({
                redirectTo: `/dashboard/modules?u=${encodeURIComponent(expectedUserId)}${focusedId ? `&focus=${encodeURIComponent(focusedId)}` : ""}`,
              });
            }}
          >
            <button
              type="submit"
              className="px-5 py-2.5 rounded-full bg-red-500/20 hover:bg-red-500/30 border border-red-500/30 text-red-200 text-sm font-semibold spring-transition cursor-pointer"
            >
              로그아웃하고 다시 시도
            </button>
          </form>
        </div>
      </div>
    );
  }

  const [allModules, entitlements] = await Promise.all([
    prisma.module.findMany({
      where: { active: true, category: { in: VISIBLE_CATEGORIES } },
      orderBy: [{ category: "asc" }, { id: "asc" }],
      select: {
        id: true,
        name: true,
        category: true,
        description: true,
        priceKrw: true,
      },
    }),
    prisma.entitlement.findMany({
      where: { userId: session.user.id },
      select: { moduleId: true, paymentId: true },
    }),
  ]);

  const ownedSet = new Set(entitlements.map(e => e.moduleId));

  return (
    <ModuleCatalog
      modules={allModules}
      ownedModuleIds={Array.from(ownedSet)}
      categoryLabels={CATEGORY_LABELS}
      visibleCategories={VISIBLE_CATEGORIES}
      focusedId={focusedId}
    />
  );
}
