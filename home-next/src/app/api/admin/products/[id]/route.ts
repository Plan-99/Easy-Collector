import { NextResponse } from "next/server";
import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";
import { parseVariants } from "../route";

async function requireAdmin() {
  const session = await auth();
  if (!session?.user?.id)
    return { error: NextResponse.json({ error: "UNAUTHORIZED" }, { status: 401 }) };
  const me = await prisma.user.findUnique({ where: { id: session.user.id } });
  if (me?.role !== "admin")
    return { error: NextResponse.json({ error: "FORBIDDEN" }, { status: 403 }) };
  return { adminId: session.user.id };
}

export async function PATCH(
  req: Request,
  ctx: { params: Promise<{ id: string }> }
) {
  const guard = await requireAdmin();
  if (guard.error) return guard.error;
  const { id } = await ctx.params;

  const body = (await req.json().catch(() => ({}))) as Record<string, unknown>;
  const data: Record<string, unknown> = {};

  // sku: empty string clears it (when variants exist), valid string sets it.
  if (typeof body.sku === "string" || body.sku === null) {
    const v = body.sku === null ? "" : String(body.sku).trim().slice(0, 100);
    data.sku = v || null;
  }
  if (typeof body.name === "string") {
    const v = body.name.trim().slice(0, 200);
    if (v) data.name = v;
  }
  if (body.description === null || typeof body.description === "string") {
    data.description = body.description ? String(body.description).slice(0, 5000) : null;
  }
  if (typeof body.priceKrw === "number" && body.priceKrw >= 0) {
    data.priceKrw = Math.floor(body.priceKrw);
  }
  if (typeof body.stock === "number" && body.stock >= 0) {
    data.stock = Math.floor(body.stock);
  }
  if (body.weightG === null || typeof body.weightG === "number") {
    data.weightG = body.weightG === null ? null : Math.max(0, Math.floor(Number(body.weightG)));
  }
  if (body.leadTimeDays === null || typeof body.leadTimeDays === "number") {
    data.leadTimeDays =
      body.leadTimeDays === null
        ? null
        : Math.max(0, Math.floor(Number(body.leadTimeDays)));
  }
  if (body.imageUrl === null || typeof body.imageUrl === "string") {
    data.imageUrl = body.imageUrl ? String(body.imageUrl).trim().slice(0, 500) : null;
  }
  if (typeof body.active === "boolean") data.active = body.active;

  // Optional variants: array of { name, sku, priceKrw, stock, active? }.
  // If `variants` is included in the body (even empty array), we replace the
  // full set. If omitted, variants are left untouched.
  const variantsField = (body as { variants?: unknown }).variants;
  const variantsProvided = Array.isArray(variantsField);
  const parsed = variantsProvided
    ? parseVariants(variantsField as Parameters<typeof parseVariants>[0])
    : { list: undefined as undefined };
  if ("error" in parsed && parsed.error) return parsed.error;

  if (Object.keys(data).length === 0 && !variantsProvided) {
    return NextResponse.json({ error: "NO_FIELDS" }, { status: 400 });
  }

  // Validate SKU requirement: if variants are being cleared / never existed,
  // a non-null Product.sku is required. If variants exist post-update, the
  // Product.sku may be null.
  if (data.sku === null) {
    const willHaveVariants = variantsProvided
      ? (parsed.list?.length || 0) > 0
      : (await prisma.productVariant.count({ where: { productId: id } })) > 0;
    if (!willHaveVariants) {
      return NextResponse.json(
        { error: "SKU_REQUIRED", hint: "옵션이 없는 상품은 기본 SKU가 필요합니다." },
        { status: 400 }
      );
    }
  }

  try {
    const product = await prisma.$transaction(async tx => {
      if (Object.keys(data).length > 0) {
        await tx.product.update({ where: { id }, data });
      }
      if (variantsProvided) {
        // Snapshot existing variants → diff by sku to keep IDs stable for
        // history. Variants no longer in the new list get marked inactive
        // if they have order references, otherwise hard-deleted.
        const existing = await tx.productVariant.findMany({ where: { productId: id } });
        const newList = parsed.list || [];
        const newSkus = new Set(newList.map(v => v.sku));

        for (const ex of existing) {
          if (!newSkus.has(ex.sku)) {
            const refCount = await tx.orderItem.count({ where: { productVariantId: ex.id } });
            if (refCount > 0) {
              await tx.productVariant.update({ where: { id: ex.id }, data: { active: false } });
            } else {
              await tx.productVariant.delete({ where: { id: ex.id } });
            }
          }
        }
        for (const v of newList) {
          const ex = existing.find(e => e.sku === v.sku);
          if (ex) {
            await tx.productVariant.update({
              where: { id: ex.id },
              data: {
                name: v.name,
                priceKrw: v.priceKrw,
                stock: v.stock,
                active: v.active,
                sortOrder: v.sortOrder,
              },
            });
          } else {
            await tx.productVariant.create({
              data: { ...v, productId: id },
            });
          }
        }
      }
      return tx.product.findUnique({ where: { id }, include: { variants: true } });
    });
    console.log(`[admin/products] ${id} updated by ${guard.adminId}`);
    return NextResponse.json({ ok: true, product });
  } catch (err: unknown) {
    const code = (err as { code?: string })?.code;
    if (code === "P2025") return NextResponse.json({ error: "NOT_FOUND" }, { status: 404 });
    if (code === "P2002") return NextResponse.json({ error: "SKU_DUPLICATE" }, { status: 409 });
    console.error("[admin/products] update failed", err);
    return NextResponse.json({ error: "UPDATE_FAILED" }, { status: 500 });
  }
}

export async function DELETE(
  _req: Request,
  ctx: { params: Promise<{ id: string }> }
) {
  const guard = await requireAdmin();
  if (guard.error) return guard.error;
  const { id } = await ctx.params;

  // Hard delete only if no order ever referenced this product. Otherwise,
  // soft-delete by deactivating so historical orders stay intact.
  const refCount = await prisma.orderItem.count({ where: { productId: id } });
  if (refCount > 0) {
    await prisma.product.update({ where: { id }, data: { active: false } });
    return NextResponse.json({ ok: true, softDeleted: true });
  }
  try {
    await prisma.product.delete({ where: { id } });
    return NextResponse.json({ ok: true, softDeleted: false });
  } catch (err: unknown) {
    if ((err as { code?: string })?.code === "P2025")
      return NextResponse.json({ error: "NOT_FOUND" }, { status: 404 });
    return NextResponse.json({ error: "DELETE_FAILED" }, { status: 500 });
  }
}
