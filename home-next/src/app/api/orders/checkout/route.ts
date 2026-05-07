import { NextResponse } from "next/server";
import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";
import { buildPaymentId } from "@/lib/portone";

// Body: { productId, quantity, address: { recipientName, phone, postalCode, line1, line2?, requestNote? } }
// Creates an Order(PENDING) + Payment(PENDING) and returns paymentId.
// Stock is checked but not yet decremented — applyPaidOrder() does that on
// payment verification.
export async function POST(req: Request) {
  const session = await auth();
  if (!session?.user?.id)
    return NextResponse.json({ error: "UNAUTHORIZED" }, { status: 401 });
  const userId = session.user.id;

  const body = (await req.json().catch(() => ({}))) as {
    productId?: string;
    variantId?: string;
    quantity?: number;
    address?: {
      recipientName?: string;
      phone?: string;
      postalCode?: string;
      line1?: string;
      line2?: string;
      requestNote?: string;
    };
  };

  const productId = (body.productId || "").trim();
  const variantId = (body.variantId || "").trim() || null;
  const quantity = Math.max(1, Math.floor(Number(body.quantity ?? 1)));
  const a = body.address || {};
  const recipientName = (a.recipientName || "").trim().slice(0, 100);
  const phone = (a.phone || "").trim().slice(0, 30);
  const postalCode = (a.postalCode || "").trim().slice(0, 10);
  const addressLine1 = (a.line1 || "").trim().slice(0, 200);
  const addressLine2 = (a.line2 || "").trim().slice(0, 200) || null;
  const requestNote = (a.requestNote || "").trim().slice(0, 500) || null;

  if (!productId) return NextResponse.json({ error: "PRODUCT_ID_REQUIRED" }, { status: 400 });
  if (!recipientName || !phone || !postalCode || !addressLine1) {
    return NextResponse.json({ error: "ADDRESS_REQUIRED" }, { status: 400 });
  }

  const product = await prisma.product.findUnique({
    where: { id: productId },
    include: { variants: { where: { active: true } } },
  });
  if (!product || !product.active) {
    return NextResponse.json({ error: "PRODUCT_NOT_FOUND" }, { status: 404 });
  }

  // If product has variants, the buyer must pick one and we use the variant's
  // price + stock instead of the product's. Otherwise fall back to product.
  let unitPriceKrw: number;
  let availableStock: number;
  let variantSnapshot: { id: string; name: string; sku: string } | null = null;

  if (product.variants.length > 0) {
    if (!variantId) {
      return NextResponse.json({ error: "VARIANT_REQUIRED" }, { status: 400 });
    }
    const v = product.variants.find(x => x.id === variantId);
    if (!v) return NextResponse.json({ error: "VARIANT_NOT_FOUND" }, { status: 404 });
    unitPriceKrw = v.priceKrw;
    availableStock = v.stock;
    variantSnapshot = { id: v.id, name: v.name, sku: v.sku };
  } else {
    unitPriceKrw = product.priceKrw;
    availableStock = product.stock;
  }

  if (unitPriceKrw <= 0) {
    return NextResponse.json({ error: "PRODUCT_NOT_PURCHASABLE" }, { status: 400 });
  }
  if (availableStock < quantity) {
    return NextResponse.json(
      { error: "INSUFFICIENT_STOCK", stock: availableStock },
      { status: 409 }
    );
  }

  const totalKrw = unitPriceKrw * quantity;
  const paymentId = buildPaymentId("ord");
  const baseName = variantSnapshot ? `${product.name} (${variantSnapshot.name})` : product.name;
  const orderName = quantity > 1 ? `${baseName} × ${quantity}` : baseName;

  const order = await prisma.$transaction(async tx => {
    const ord = await tx.order.create({
      data: {
        userId: userId,
        status: "PENDING",
        totalKrw,
        recipientName,
        phone,
        postalCode,
        addressLine1,
        addressLine2,
        requestNote,
        items: {
          create: [
            {
              productId: product.id,
              productName: product.name,
              // Product.sku is nullable; for variant-using products it's
              // empty and the variant SKU is the actual fulfillment id.
              productSku: product.sku ?? "",
              productVariantId: variantSnapshot?.id || null,
              variantName: variantSnapshot?.name || null,
              variantSku: variantSnapshot?.sku || null,
              quantity,
              unitPriceKrw,
            },
          ],
        },
      },
    });
    await tx.payment.create({
      data: {
        userId: userId,
        orderId: ord.id,
        portonePaymentId: paymentId,
        status: "PENDING",
        amountKrw: totalKrw,
      },
    });
    return ord;
  });

  return NextResponse.json({
    paymentId,
    orderId: order.id,
    amountKrw: totalKrw,
    orderName: orderName.length > 60 ? orderName.slice(0, 57) + "…" : orderName,
  });
}
