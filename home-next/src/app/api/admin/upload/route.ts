import { NextResponse } from "next/server";
import { put } from "@vercel/blob";
import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";

// Admin-only image uploader. Streams the request body straight to Vercel Blob
// and returns the public URL. Client uses the URL as Product.imageUrl.
//
// Requires BLOB_READ_WRITE_TOKEN env (auto-provisioned when you connect a
// Vercel Blob store to the project).
export async function POST(req: Request) {
  const session = await auth();
  if (!session?.user?.id)
    return NextResponse.json({ error: "UNAUTHORIZED" }, { status: 401 });
  const me = await prisma.user.findUnique({ where: { id: session.user.id } });
  if (me?.role !== "admin")
    return NextResponse.json({ error: "FORBIDDEN" }, { status: 403 });

  const url = new URL(req.url);
  const filename = url.searchParams.get("filename");
  if (!filename) return NextResponse.json({ error: "FILENAME_REQUIRED" }, { status: 400 });

  if (!req.body) return NextResponse.json({ error: "NO_BODY" }, { status: 400 });

  // 5 MB cap to keep thumbnails reasonable
  const contentLength = Number(req.headers.get("content-length") || 0);
  if (contentLength > 5 * 1024 * 1024) {
    return NextResponse.json({ error: "FILE_TOO_LARGE", maxBytes: 5 * 1024 * 1024 }, { status: 413 });
  }

  // Sanitize: keep extension, replace spaces, prefix with timestamp for uniqueness
  const safe = filename
    .replace(/[^\w.\-]+/g, "_")
    .slice(-100);
  const key = `products/${Date.now()}_${safe}`;

  try {
    const blob = await put(key, req.body, {
      access: "public",
      contentType: req.headers.get("content-type") || undefined,
    });
    return NextResponse.json({ url: blob.url, pathname: blob.pathname });
  } catch (err) {
    console.error("[admin/upload] blob put failed", err);
    if (err instanceof Error && /BLOB_READ_WRITE_TOKEN/i.test(err.message)) {
      return NextResponse.json(
        { error: "BLOB_NOT_CONFIGURED", hint: "Create a Vercel Blob store and connect it to this project." },
        { status: 503 }
      );
    }
    return NextResponse.json({ error: "UPLOAD_FAILED" }, { status: 500 });
  }
}
