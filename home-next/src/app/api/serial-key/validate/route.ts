import { prisma } from "@/lib/prisma";
import { NextRequest } from "next/server";

export async function POST(request: NextRequest) {
  const body = await request.json();
  const { key, machineId } = body;

  if (!key || !machineId) {
    return Response.json(
      { valid: false, error: "key and machineId required" },
      { status: 400 }
    );
  }

  const serialKey = await prisma.serialKey.findUnique({
    where: { key },
    include: { user: true },
  });

  if (!serialKey) {
    return Response.json({ valid: false, error: "Invalid serial key" }, { status: 404 });
  }

  if (!serialKey.active) {
    return Response.json({ valid: false, error: "Serial key is deactivated" }, { status: 403 });
  }

  // First activation: bind to this machine
  if (!serialKey.machineId) {
    await prisma.serialKey.update({
      where: { id: serialKey.id },
      data: { machineId, lastActiveAt: new Date() },
    });
    return Response.json({
      valid: true,
      plan: serialKey.plan,
      message: "Serial key activated and bound to this machine",
    });
  }

  // Already bound: check machine matches
  if (serialKey.machineId !== machineId) {
    return Response.json(
      { valid: false, error: "Serial key is bound to a different machine" },
      { status: 403 }
    );
  }

  // Update last active time
  await prisma.serialKey.update({
    where: { id: serialKey.id },
    data: { lastActiveAt: new Date() },
  });

  return Response.json({
    valid: true,
    plan: serialKey.plan,
  });
}
