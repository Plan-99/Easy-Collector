import { NextRequest } from "next/server";
import { prisma } from "@/lib/prisma";

// Step 3 of OAuth-device-flow: launcher polls this every couple of seconds.
// Returns one of:
//   { status: "PENDING" }                                       — keep polling
//   { status: "APPROVED", accessToken, user, device }           — success, stop polling
//   { status: "REJECTED" | "EXPIRED", errorCode? }              — fail, stop polling
//
// On APPROVED, the request row is marked CONSUMED so the token can only be
// retrieved once — after that, the launcher must re-run /start.

export async function GET(req: NextRequest) {
  const url = new URL(req.url);
  const nonce = url.searchParams.get("nonce");
  if (!nonce) return Response.json({ error: "nonce required" }, { status: 400 });

  const reqRow = await prisma.deviceAuthRequest.findUnique({ where: { nonce } });
  if (!reqRow) return Response.json({ error: "unknown nonce" }, { status: 404 });

  if (reqRow.status === "PENDING" && reqRow.expiresAt < new Date()) {
    await prisma.deviceAuthRequest.update({
      where: { nonce },
      data: { status: "EXPIRED" },
    });
    return Response.json({ status: "EXPIRED" });
  }

  if (reqRow.status === "PENDING") return Response.json({ status: "PENDING" });
  if (reqRow.status === "REJECTED")
    return Response.json({ status: "REJECTED", errorCode: reqRow.errorCode });
  if (reqRow.status === "EXPIRED") return Response.json({ status: "EXPIRED" });
  if (reqRow.status === "CONSUMED")
    // Token already retrieved — nonce is single-use.
    return Response.json({ status: "REJECTED", errorCode: "ALREADY_CONSUMED" });

  // status === "APPROVED" → return the device token, then mark CONSUMED.
  if (!reqRow.deviceId || !reqRow.userId) {
    // Defensive: an APPROVED row should always have these set.
    await prisma.deviceAuthRequest.update({
      where: { nonce },
      data: { status: "REJECTED", errorCode: "INTERNAL" },
    });
    return Response.json({ status: "REJECTED", errorCode: "INTERNAL" });
  }

  const device = await prisma.device.findUnique({
    where: { id: reqRow.deviceId },
    include: { user: true },
  });
  if (!device || !device.active) {
    return Response.json({ status: "REJECTED", errorCode: "DEVICE_INACTIVE" });
  }

  await prisma.deviceAuthRequest.update({
    where: { nonce },
    data: { status: "CONSUMED" },
  });

  return Response.json({
    status: "APPROVED",
    accessToken: device.accessToken,
    user: {
      id: device.user.id,
      email: device.user.email,
      name: device.user.name,
      image: device.user.image,
      role: device.user.role,
      plan: device.user.plan,
    },
    device: {
      id: device.id,
      machineId: device.machineId,
      hostname: device.hostname,
      os: device.os,
    },
  });
}
