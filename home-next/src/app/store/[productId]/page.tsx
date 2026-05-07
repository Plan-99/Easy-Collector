import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";
import { notFound, redirect } from "next/navigation";
import Link from "next/link";
import Footer from "@/components/Footer";
import Navbar from "@/components/Navbar";
import OrderForm from "./OrderForm";
import LoginGate from "./LoginGate";
import {
  PORTONE_STORE_ID,
  PORTONE_CHANNEL_KEY,
} from "@/lib/portone";

export const dynamic = "force-dynamic";

function formatKrw(amountKrw: number) {
  return new Intl.NumberFormat("ko-KR", {
    style: "currency",
    currency: "KRW",
    maximumFractionDigits: 0,
  }).format(amountKrw);
}

export default async function StoreProductPage({
  params,
}: {
  params: Promise<{ productId: string }>;
}) {
  const { productId } = await params;
  const product = await prisma.product.findUnique({
    where: { id: productId },
    include: {
      variants: {
        where: { active: true },
        orderBy: { sortOrder: "asc" },
      },
    },
  });
  if (!product || !product.active) notFound();

  const session = await auth();
  const isAuthed = !!session?.user?.id;

  // Logged-in user data is fetched only when authenticated. Onboarding is
  // still required for actual purchase — we redirect there only if the user
  // is signed in but hasn't completed it.
  const me =
    isAuthed && session?.user?.id
      ? await prisma.user.findUnique({
          where: { id: session.user.id },
          select: {
            organization: true,
            department: true,
            jobRole: true,
            termsAcceptedAt: true,
            privacyAcceptedAt: true,
            refundAcceptedAt: true,
            addresses: {
              orderBy: [{ isDefault: "desc" }, { createdAt: "desc" }] as const,
            },
          },
        })
      : null;
  if (me) {
    const onboarded =
      !!me.organization &&
      !!me.department &&
      !!me.jobRole &&
      !!me.termsAcceptedAt &&
      !!me.privacyAcceptedAt &&
      !!me.refundAcceptedAt;
    if (!onboarded) {
      redirect(`/onboarding?next=${encodeURIComponent(`/store/${productId}`)}`);
    }
  }

  const configMissing = !PORTONE_STORE_ID || !PORTONE_CHANNEL_KEY;
  const hasVariants = product.variants.length > 0;
  const totalStock = hasVariants
    ? product.variants.reduce((s, v) => s + v.stock, 0)
    : product.stock;
  const minPrice = hasVariants
    ? product.variants.reduce((m, v) => (v.priceKrw < m ? v.priceKrw : m), product.variants[0].priceKrw)
    : product.priceKrw;

  return (
    <div className="min-h-dvh flex flex-col bg-surface-950">
      <Navbar
        isLoggedIn={isAuthed}
        isAdmin={(session?.user as { role?: string })?.role === "admin"}
      />
      <div className="flex-1 px-6 pt-24 pb-12">
        <div className="max-w-3xl mx-auto">
          <Link href="/store" className="text-xs text-surface-400 hover:text-white underline">
            ← 스토어
          </Link>

          <div className="bezel-card mt-4">
            <div className="bezel-inner p-8">
              <div className="flex gap-6">
                <div className="shrink-0 w-44 h-44 rounded-xl bg-white/5 border border-white/10 flex items-center justify-center overflow-hidden">
                  {product.imageUrl ? (
                    // eslint-disable-next-line @next/next/no-img-element
                    <img
                      src={product.imageUrl}
                      alt={product.name}
                      className="w-full h-full object-contain p-3"
                    />
                  ) : (
                    <span className="text-surface-600 text-xs">이미지 없음</span>
                  )}
                </div>
                <div className="min-w-0 flex-1">
                  <h1 className="font-bold text-2xl text-surface-100">{product.name}</h1>
                  {product.sku && (
                    <p className="text-surface-500 text-xs mt-1 font-mono">{product.sku}</p>
                  )}
                  <p className="font-mono font-bold text-3xl mt-4">
                    {hasVariants && minPrice !== product.variants[0].priceKrw && (
                      <span className="text-surface-500 text-sm font-normal">
                        from{" "}
                      </span>
                    )}
                    {formatKrw(minPrice)}
                  </p>
                  <div className="flex items-center gap-4 mt-3 text-xs text-surface-400">
                    {!hasVariants &&
                      (product.stock > 0 ? (
                        <span>재고 {product.stock}개</span>
                      ) : (
                        <span className="text-amber-300">품절</span>
                      ))}
                    {product.leadTimeDays != null && (
                      <span>발송 소요 {product.leadTimeDays}일</span>
                    )}
                  </div>
                </div>
              </div>
              {product.description && (
                <p className="text-surface-300 text-sm mt-6 leading-relaxed whitespace-pre-line">
                  {product.description}
                </p>
              )}
            </div>
          </div>

          {!isAuthed ? (
            <LoginGate
              callbackUrl={`/store/${productId}`}
              autoOpen
            />
          ) : totalStock > 0 ? (
            configMissing ? (
              <div className="bezel-card mt-4">
                <div className="bezel-inner p-8 text-center text-amber-300">
                  결제 설정이 누락되어 주문할 수 없습니다.
                </div>
              </div>
            ) : (
              <OrderForm
                productId={product.id}
                productName={product.name}
                priceKrw={product.priceKrw}
                maxQuantity={hasVariants ? 0 : product.stock}
                variants={product.variants.map(v => ({
                  id: v.id,
                  name: v.name,
                  priceKrw: v.priceKrw,
                  stock: v.stock,
                }))}
                savedAddresses={(me?.addresses || []).map(a => ({
                  id: a.id,
                  label: a.label,
                  recipientName: a.recipientName,
                  phone: a.phone,
                  postalCode: a.postalCode,
                  addressLine1: a.addressLine1,
                  addressLine2: a.addressLine2,
                  requestNote: a.requestNote,
                  isDefault: a.isDefault,
                }))}
                buyerEmail={session!.user!.email || ""}
                buyerName={session!.user!.name || ""}
                customerId={session!.user!.id!.slice(0, 20)}
                storeId={PORTONE_STORE_ID}
                channelKey={PORTONE_CHANNEL_KEY}
              />
            )
          ) : (
            <div className="bezel-card mt-4">
              <div className="bezel-inner p-8 text-center text-amber-300 text-sm">
                현재 품절입니다. 입고 후 다시 오픈됩니다.
              </div>
            </div>
          )}
        </div>
      </div>
      <Footer />
    </div>
  );
}
