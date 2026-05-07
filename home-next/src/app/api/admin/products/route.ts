import { NextResponse } from "next/server";
import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";

async function requireAdmin() {
  const session = await auth();
  if (!session?.user?.id)
    return { error: NextResponse.json({ error: "UNAUTHORIZED" }, { status: 401 }) };
  const me = await prisma.user.findUnique({ where: { id: session.user.id } });
  if (me?.role !== "admin")
    return { error: NextResponse.json({ error: "FORBIDDEN" }, { status: 403 }) };
  return { adminId: session.user.id };
}

type Body = Partial<{
  sku: string;
  name: string;
  description: string | null;
  priceKrw: number;
  stock: number;
  weightG: number | null;
  leadTimeDays: number | null;
  imageUrl: string | null;
  active: boolean;
}>;

function buildData(body: Body, requireAll = false) {
  const data: Record<string, unknown> = {};
  if (body.sku !== undefined) {
    const raw = String(body.sku ?? "").trim().slice(0, 100);
    // Empty SKU is allowed — variant-using products can leave it blank.
    data.sku = raw || null;
  }
  if (body.name !== undefined) {
    const v = String(body.name).trim().slice(0, 200);
    if (!v && requireAll) throw new Error("NAME_REQUIRED");
    if (v) data.name = v;
  }
  if (body.description !== undefined) {
    data.description = body.description ? String(body.description).slice(0, 5000) : null;
  }
  if (body.priceKrw !== undefined) {
    const v = Number(body.priceKrw);
    if (!Number.isFinite(v) || v < 0) throw new Error("INVALID_PRICE");
    data.priceKrw = Math.floor(v);
  }
  if (body.stock !== undefined) {
    const v = Number(body.stock);
    if (!Number.isFinite(v) || v < 0) throw new Error("INVALID_STOCK");
    data.stock = Math.floor(v);
  }
  if (body.weightG !== undefined) {
    data.weightG = body.weightG === null ? null : Math.max(0, Math.floor(Number(body.weightG)));
  }
  if (body.leadTimeDays !== undefined) {
    data.leadTimeDays =
      body.leadTimeDays === null ? null : Math.max(0, Math.floor(Number(body.leadTimeDays)));
  }
  if (body.imageUrl !== undefined) {
    data.imageUrl = body.imageUrl ? String(body.imageUrl).trim().slice(0, 500) : null;
  }
  if (body.active !== undefined) data.active = !!body.active;
  return data;
}

export async function POST(req: Request) {
  const guard = await requireAdmin();
  if (guard.error) return guard.error;

  const body = (await req.json().catch(() => ({}))) as Body;
  let data: Record<string, unknown>;
  try {
    data = buildData(body, true);
  } catch (err) {
    return NextResponse.json({ error: (err as Error).message }, { status: 400 });
  }
  if (!data.name || data.priceKrw === undefined) {
    return NextResponse.json({ error: "MISSING_FIELDS" }, { status: 400 });
  }
  // Optional variants: array of { name, sku, priceKrw, stock, active? }
  const variantsBody = (body as Body & { variants?: VariantInput[] }).variants;
  const variantCreate = parseVariants(variantsBody);
  if (variantCreate.error) return variantCreate.error;
  // SKU is required only when there are no variants (variant SKUs identify
  // shippable units in that case).
  const variantCount = variantCreate.list?.length || 0;
  if (variantCount === 0 && !data.sku) {
    return NextResponse.json(
      { error: "SKU_REQUIRED", hint: "옵션이 없는 상품은 기본 SKU가 필요합니다." },
      { status: 400 }
    );
  }

  try {
    const product = await prisma.product.create({
      data: {
        ...(data as Parameters<typeof prisma.product.create>[0]["data"]),
        ...(variantCreate.list && variantCreate.list.length > 0
          ? { variants: { create: variantCreate.list } }
          : {}),
      },
    });
    return NextResponse.json({ ok: true, product });
  } catch (err: unknown) {
    if ((err as { code?: string })?.code === "P2002") {
      return NextResponse.json({ error: "SKU_DUPLICATE" }, { status: 409 });
    }
    console.error("[admin/products] create failed", err);
    return NextResponse.json({ error: "CREATE_FAILED" }, { status: 500 });
  }
}

type VariantInput = {
  name?: string;
  sku?: string;
  priceKrw?: number;
  stock?: number;
  active?: boolean;
  sortOrder?: number;
};

export function parseVariants(input: VariantInput[] | undefined): {
  error?: NextResponse;
  list?: {
    name: string;
    sku: string;
    priceKrw: number;
    stock: number;
    active: boolean;
    sortOrder: number;
  }[];
} {
  if (!Array.isArray(input)) return { list: [] };
  const list: {
    name: string;
    sku: string;
    priceKrw: number;
    stock: number;
    active: boolean;
    sortOrder: number;
  }[] = [];
  for (let i = 0; i < input.length; i++) {
    const v = input[i] || {};
    const name = String(v.name || "").trim().slice(0, 200);
    const sku = String(v.sku || "").trim().slice(0, 100);
    if (!name && !sku) continue; // skip empty rows
    if (!name || !sku) {
      return {
        error: NextResponse.json(
          { error: "VARIANT_INVALID", index: i, hint: "name + sku required" },
          { status: 400 }
        ),
      };
    }
    const priceKrw = Number(v.priceKrw);
    if (!Number.isFinite(priceKrw) || priceKrw < 0) {
      return {
        error: NextResponse.json(
          { error: "VARIANT_INVALID_PRICE", index: i },
          { status: 400 }
        ),
      };
    }
    const stock = Math.max(0, Math.floor(Number(v.stock ?? 0)));
    list.push({
      name,
      sku,
      priceKrw: Math.floor(priceKrw),
      stock,
      active: v.active !== false,
      sortOrder: typeof v.sortOrder === "number" ? v.sortOrder : i,
    });
  }
  return { list };
}
