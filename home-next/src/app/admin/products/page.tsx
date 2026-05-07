import { prisma } from "@/lib/prisma";
import Link from "next/link";

function formatKrw(amountKrw: number) {
  return new Intl.NumberFormat("ko-KR", {
    style: "currency",
    currency: "KRW",
    maximumFractionDigits: 0,
  }).format(amountKrw);
}

export default async function AdminProductsPage() {
  const products = await prisma.product.findMany({
    orderBy: [{ active: "desc" }, { createdAt: "desc" }],
  });

  return (
    <>
      <div className="flex items-center justify-between mb-4">
        <p className="text-surface-400 text-sm">총 {products.length}개 상품</p>
        <Link
          href="/admin/products/new"
          className="px-4 py-2 rounded-full bg-indigo-500/20 hover:bg-indigo-500/30 border border-indigo-500/30 text-indigo-200 text-xs font-semibold spring-transition cursor-pointer"
        >
          + 상품 추가
        </Link>
      </div>
      <div className="bezel-card">
        <div className="bezel-inner p-0 overflow-hidden">
          <table className="w-full text-sm">
            <thead>
              <tr className="border-b border-white/10 text-left text-surface-400">
                <th className="px-5 py-3 font-medium">상품</th>
                <th className="px-5 py-3 font-medium">SKU</th>
                <th className="px-5 py-3 font-medium text-right">가격</th>
                <th className="px-5 py-3 font-medium text-right">재고</th>
                <th className="px-5 py-3 font-medium text-center">활성</th>
                <th className="px-5 py-3 font-medium text-right">편집</th>
              </tr>
            </thead>
            <tbody>
              {products.map(p => (
                <tr key={p.id} className="border-b border-white/5 hover:bg-white/[0.02]">
                  <td className="px-5 py-3">
                    <div className="flex items-center gap-3">
                      {p.imageUrl ? (
                        // eslint-disable-next-line @next/next/no-img-element
                        <img
                          src={p.imageUrl}
                          alt={p.name}
                          className="w-10 h-10 object-contain rounded bg-white/5 p-1"
                        />
                      ) : (
                        <div className="w-10 h-10 rounded bg-white/5" />
                      )}
                      <span className="font-medium text-surface-100">{p.name}</span>
                    </div>
                  </td>
                  <td className="px-5 py-3 font-mono text-xs text-surface-400">
                    {p.sku || <span className="text-surface-600">옵션별</span>}
                  </td>
                  <td className="px-5 py-3 text-right font-mono">{formatKrw(p.priceKrw)}</td>
                  <td className="px-5 py-3 text-right">
                    <span
                      className={`text-xs font-semibold ${
                        p.stock > 0 ? "text-surface-200" : "text-amber-300"
                      }`}
                    >
                      {p.stock}
                    </span>
                  </td>
                  <td className="px-5 py-3 text-center">
                    <span
                      className={`text-xs font-semibold px-2 py-0.5 rounded ${
                        p.active ? "bg-emerald-500/20 text-emerald-300" : "bg-surface-800 text-surface-500"
                      }`}
                    >
                      {p.active ? "ON" : "OFF"}
                    </span>
                  </td>
                  <td className="px-5 py-3 text-right">
                    <Link
                      href={`/admin/products/${p.id}`}
                      className="text-xs text-indigo-300 hover:text-indigo-200 underline"
                    >
                      편집
                    </Link>
                  </td>
                </tr>
              ))}
              {products.length === 0 && (
                <tr>
                  <td colSpan={6} className="px-5 py-8 text-center text-surface-500 text-sm">
                    등록된 상품이 없습니다. "+ 상품 추가"로 첫 상품을 만드세요.
                  </td>
                </tr>
              )}
            </tbody>
          </table>
        </div>
      </div>
    </>
  );
}
