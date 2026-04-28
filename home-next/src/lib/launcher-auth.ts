// Bearer-token authentication for the desktop launcher.
// The launcher receives an `accessToken` after the device-auth flow completes
// and sends it as `Authorization: Bearer <token>` on every API call.

import { prisma } from "./prisma";
import { NextRequest } from "next/server";

export type LauncherAuth = {
  device: {
    id: string;
    machineId: string;
    hostname: string | null;
    os: string | null;
  };
  user: {
    id: string;
    email: string;
    name: string | null;
    image: string | null;
    role: string;
    plan: string;
  };
};

function extractBearer(req: NextRequest): string | null {
  const header = req.headers.get("authorization") || req.headers.get("Authorization");
  if (!header) return null;
  const parts = header.split(/\s+/);
  if (parts.length !== 2 || parts[0].toLowerCase() !== "bearer") return null;
  return parts[1].trim() || null;
}

export async function authenticateLauncher(req: NextRequest): Promise<LauncherAuth | null> {
  const token = extractBearer(req);
  if (!token) return null;

  const device = await prisma.device.findUnique({
    where: { accessToken: token },
    include: { user: true },
  });
  if (!device || !device.active) return null;

  // Update lastSeenAt opportunistically; ignore failures.
  prisma.device
    .update({ where: { id: device.id }, data: { lastSeenAt: new Date() } })
    .catch(() => {});

  return {
    device: {
      id: device.id,
      machineId: device.machineId,
      hostname: device.hostname,
      os: device.os,
    },
    user: {
      id: device.user.id,
      email: device.user.email,
      name: device.user.name,
      image: device.user.image,
      role: device.user.role,
      plan: device.user.plan,
    },
  };
}

export function unauthorized(message = "Unauthorized") {
  return Response.json({ error: message }, { status: 401 });
}
