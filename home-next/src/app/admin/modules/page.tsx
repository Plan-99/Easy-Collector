import { prisma } from "@/lib/prisma";
import ModuleRow from "./ModuleRow";

export const dynamic = "force-dynamic";

export default async function AdminModulesPage() {
  const modules = await prisma.module.findMany({
    orderBy: [{ category: "asc" }, { id: "asc" }],
  });

  return (
    <>
      <p className="text-surface-400 text-sm mb-4">
        총 {modules.length}개 모듈 — 가격은 KRW(원) 기준. 0원으로 설정하면 무료 모듈로 노출됩니다.
      </p>
      <div className="bezel-card">
        <div className="bezel-inner p-0 overflow-hidden">
          <table className="w-full text-sm">
            <thead>
              <tr className="border-b border-white/10 text-left text-surface-400">
                <th className="px-5 py-3 font-medium">ID</th>
                <th className="px-5 py-3 font-medium">이름</th>
                <th className="px-5 py-3 font-medium">카테고리</th>
                <th className="px-5 py-3 font-medium text-right">가격 (₩)</th>
                <th className="px-5 py-3 font-medium text-center">활성</th>
                <th className="px-5 py-3 font-medium text-right">저장</th>
              </tr>
            </thead>
            <tbody>
              {modules.map(m => (
                <ModuleRow
                  key={m.id}
                  initial={{
                    id: m.id,
                    name: m.name,
                    category: m.category,
                    priceKrw: m.priceKrw,
                    active: m.active,
                  }}
                />
              ))}
            </tbody>
          </table>
        </div>
      </div>
    </>
  );
}
