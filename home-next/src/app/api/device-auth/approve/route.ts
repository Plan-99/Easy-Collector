import { NextRequest } from "next/server";
import { prisma } from "@/lib/prisma";
import { auth } from "@/auth";
import { randomBytes } from "crypto";

// Step 2.5: web page calls this after the user clicks "이 기기 등록".
// Requires an active NextAuth session (Google login completed).
//
// Body: { nonce: string, action: "approve" | "reject" }
//
// Approve flow:
//   1. Verify the nonce is PENDING and not expired
//   2. Verify the user has no other active Device → if they do, REJECT with
//      DEVICE_ALREADY_BOUND (the "이미 사용중인 기기가 있습니다" rule)
//   3. Create a Device row with a fresh accessToken
//   4. Mark the request APPROVED with the new deviceId / userId

function newAccessToken() {
  // 32 bytes -> ~43 char base64url. Long enough that brute force is not feasible.
  return randomBytes(32).toString("base64url");
}

export async function POST(req: NextRequest) {
  const session = await auth();
  if (!session?.user?.id) {
    return Response.json({ error: "not signed in" }, { status: 401 });
  }
  const userId = session.user.id;

  let body: { nonce?: string; action?: string };
  try {
    body = await req.json();
  } catch {
    return Response.json({ error: "invalid json" }, { status: 400 });
  }
  const nonce = (body.nonce || "").trim();
  const action = body.action === "reject" ? "reject" : "approve";
  if (!nonce) return Response.json({ error: "nonce required" }, { status: 400 });

  const reqRow = await prisma.deviceAuthRequest.findUnique({ where: { nonce } });
  if (!reqRow) return Response.json({ error: "unknown nonce" }, { status: 404 });

  if (reqRow.status !== "PENDING") {
    return Response.json({ error: `request is ${reqRow.status}` }, { status: 409 });
  }
  if (reqRow.expiresAt < new Date()) {
    await prisma.deviceAuthRequest.update({
      where: { nonce },
      data: { status: "EXPIRED" },
    });
    return Response.json({ error: "expired" }, { status: 410 });
  }

  if (action === "reject") {
    await prisma.deviceAuthRequest.update({
      where: { nonce },
      data: { status: "REJECTED", userId, errorCode: "USER_REJECTED" },
    });
    return Response.json({ ok: true, status: "REJECTED" });
  }

  // Free plan = single active Device. Unlimited / Business plans skip the
  // bind check entirely so the user can register multiple PCs at once.
  const me = await prisma.user.findUnique({
    where: { id: userId },
    select: { plan: true },
  });
  const unlimited = me?.plan === "unlimited" || me?.plan === "business";

  if (!unlimited) {
    const existing = await prisma.device.findFirst({
      where: { userId, active: true },
      select: { id: true, machineId: true, hostname: true },
    });

    // Same machine re-binding to the same user is fine — re-issue the token.
    // But a different machineId on top of an existing active Device is rejected.
    if (existing && existing.machineId !== reqRow.machineId) {
      await prisma.deviceAuthRequest.update({
        where: { nonce },
        data: { status: "REJECTED", userId, errorCode: "DEVICE_ALREADY_BOUND" },
      });
      return Response.json(
        {
          ok: false,
          status: "REJECTED",
          errorCode: "DEVICE_ALREADY_BOUND",
          existingDevice: { hostname: existing.hostname },
        },
        { status: 409 }
      );
    }
  }

  // The same machineId might already be tied to a *different* user's Device.
  // In that case we steal the binding (the new user explicitly approved it),
  // because the login itself proves intent to take over this machine.
  await prisma.device.deleteMany({
    where: { machineId: reqRow.machineId, NOT: { userId } },
  });

  const accessToken = newAccessToken();
  const device = await prisma.device.upsert({
    where: { machineId: reqRow.machineId },
    update: {
      userId,
      accessToken,
      active: true,
      hostname: reqRow.hostname,
      os: reqRow.os,
      lastSeenAt: new Date(),
    },
    create: {
      userId,
      machineId: reqRow.machineId,
      hostname: reqRow.hostname,
      os: reqRow.os,
      accessToken,
      active: true,
    },
  });

  await prisma.deviceAuthRequest.update({
    where: { nonce },
    data: {
      status: "APPROVED",
      userId,
      deviceId: device.id,
    },
  });

  return Response.json({ ok: true, status: "APPROVED" });
}
