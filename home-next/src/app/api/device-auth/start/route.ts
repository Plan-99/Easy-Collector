import { NextRequest } from "next/server";
import { prisma } from "@/lib/prisma";
import { randomBytes } from "crypto";

// Step 1 of OAuth-device-flow: launcher asks for a nonce and a verification URL.
// User opens the URL in a browser, signs in with Google, and approves the binding.
//
// Body: { machineId: string, hostname?: string, os?: string }
// Returns: { nonce, verificationUrl, expiresAt }

const NONCE_TTL_MIN = 10;

function newNonce() {
  // 24 bytes -> 32 chars base64url. Plenty of entropy, fits in URL.
  return randomBytes(24).toString("base64url");
}

function siteOrigin(req: NextRequest) {
  // Trust the request origin in dev, but prefer NEXTAUTH_URL / NEXT_PUBLIC_SITE_URL
  // in production so the launcher always lands on the canonical host.
  return (
    process.env.NEXT_PUBLIC_SITE_URL?.replace(/\/$/, "") ||
    process.env.NEXTAUTH_URL?.replace(/\/$/, "") ||
    new URL(req.url).origin
  );
}

export async function POST(req: NextRequest) {
  let body: { machineId?: string; hostname?: string; os?: string };
  try {
    body = await req.json();
  } catch {
    return Response.json({ error: "invalid json" }, { status: 400 });
  }

  const machineId = (body.machineId || "").trim();
  if (!machineId) {
    return Response.json({ error: "machineId required" }, { status: 400 });
  }

  const nonce = newNonce();
  const expiresAt = new Date(Date.now() + NONCE_TTL_MIN * 60 * 1000);

  await prisma.deviceAuthRequest.create({
    data: {
      nonce,
      machineId,
      hostname: body.hostname || null,
      os: body.os || null,
      status: "PENDING",
      expiresAt,
    },
  });

  const origin = siteOrigin(req);
  const verificationUrl = `${origin}/auth/device?code=${encodeURIComponent(nonce)}`;

  return Response.json({
    nonce,
    verificationUrl,
    expiresAt: expiresAt.toISOString(),
    pollIntervalSec: 2,
  });
}
