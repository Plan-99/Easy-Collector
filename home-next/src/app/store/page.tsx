import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";
import Link from "next/link";
import Footer from "@/components/Footer";
import Navbar from "@/components/Navbar";

export const dynamic = "force-dynamic";

function formatKrw(amountKrw: number) {
  return new Intl.NumberFormat("ko-KR", {
    style: "currency",
    currency: "KRW",
    maximumFractionDigits: 0,
  }).format(amountKrw);
}

export default async function StorePage() {
  const session = await auth();
  const products = await prisma.product.findMany({
    where: { active: true },
    orderBy: { createdAt: "desc" },
  });

  return (
    <div className="min-h-dvh flex flex-col bg-surface-950">
      <Navbar
        isLoggedIn={!!session?.user}
        isAdmin={(session?.user as { role?: string })?.role === "admin"}
      />
      <div className="flex-1 px-6 pt-24 pb-16">
        <div className="max-w-5xl mx-auto">
          <h1 className="font-[var(--font-display)] font-bold text-3xl mb-3">하드웨어 스토어</h1>
          <p className="text-surface-400 text-sm mb-8">
            로봇 티칭에 필요한 리더 로봇·센서 등 하드웨어를 판매합니다. 결제 후
            영업일 기준 일정 안에 발송됩니다.
          </p>

          {products.length === 0 ? (
            <div className="bezel-card">
              <div className="bezel-inner p-8 text-center text-surface-500 text-sm">
                준비 중인 상품입니다.
              </div>
            </div>
          ) : (
            <div className="grid md:grid-cols-2 gap-4">
              {products.map(p => (
                <Link
                  key={p.id}
                  href={`/store/${p.id}`}
                  className="bezel-card spring-transition hover:-translate-y-1"
                >
                  <div className="bezel-inner p-6 flex gap-5">
                    <div className="shrink-0 w-28 h-28 rounded-xl bg-white/5 border border-white/10 flex items-center justify-center overflow-hidden">
                      {p.imageUrl ? (
                        // eslint-disable-next-line @next/next/no-img-element
                        <img
                          src={p.imageUrl}
                          alt={p.name}
                          className="w-full h-full object-contain p-2"
                        />
                      ) : (
                        <span className="text-surface-600 text-xs">이미지 없음</span>
                      )}
                    </div>
                    <div className="min-w-0 flex-1 flex flex-col justify-between">
                      <div>
                        <h2 className="font-bold text-lg text-surface-100 truncate">{p.name}</h2>
                        {p.description && (
                          <p className="text-surface-400 text-sm mt-1 line-clamp-2">
                            {p.description}
                          </p>
                        )}
                      </div>
                      <div className="flex items-end justify-between mt-3">
                        <p className="font-mono font-bold text-xl">{formatKrw(p.priceKrw)}</p>
                      </div>
                    </div>
                  </div>
                </Link>
              ))}
            </div>
          )}
        </div>
      </div>
      <Footer />
    </div>
  );
}
