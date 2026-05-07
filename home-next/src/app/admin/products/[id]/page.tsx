import { prisma } from "@/lib/prisma";
import { notFound } from "next/navigation";
import ProductForm from "../ProductForm";

export default async function AdminEditProductPage({
  params,
}: {
  params: Promise<{ id: string }>;
}) {
  const { id } = await params;
  const p = await prisma.product.findUnique({
    where: { id },
    include: { variants: { orderBy: { sortOrder: "asc" } } },
  });
  if (!p) notFound();

  return (
    <>
      <h2 className="font-bold text-lg mb-4">상품 편집 — {p.name}</h2>
      <ProductForm
        initial={{
          id: p.id,
          sku: p.sku || "",
          name: p.name,
          description: p.description || "",
          priceKrw: p.priceKrw,
          stock: p.stock,
          weightG: p.weightG,
          leadTimeDays: p.leadTimeDays,
          imageUrl: p.imageUrl || "",
          active: p.active,
          variants: p.variants.map(v => ({
            name: v.name,
            sku: v.sku,
            priceKrw: v.priceKrw,
            stock: v.stock,
            active: v.active,
          })),
        }}
      />
    </>
  );
}
