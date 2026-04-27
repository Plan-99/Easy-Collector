import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";
import { NextRequest, NextResponse } from "next/server";

export async function POST(
  _req: NextRequest,
  { params }: { params: Promise<{ id: string }> }
) {
  const session = await auth();
  if (!session?.user?.id) {
    return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
  }

  const user = await prisma.user.findUnique({
    where: { id: session.user.id },
  });
  if (user?.role !== "admin") {
    return NextResponse.json({ error: "Forbidden" }, { status: 403 });
  }

  const { id } = await params;

  try {
    const updated = await prisma.serialKey.update({
      where: { id },
      data: { machineId: null, lastActiveAt: null },
    });
    return NextResponse.json({ success: true, key: updated.key });
  } catch {
    return NextResponse.json(
      { error: "Serial key not found" },
      { status: 404 }
    );
  }
}
