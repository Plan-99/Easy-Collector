import { NextResponse } from "next/server";
import bcrypt from "bcryptjs";
import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";

// POST   { newPassword, currentPassword? } — set or change password.
//        currentPassword is required if user already has a password.
// DELETE                                  — remove password (Google-only login).
export async function POST(req: Request) {
  const session = await auth();
  if (!session?.user?.id) {
    return NextResponse.json({ error: "UNAUTHORIZED" }, { status: 401 });
  }

  const body = (await req.json().catch(() => ({}))) as {
    newPassword?: string;
    currentPassword?: string;
  };
  const newPassword = String(body.newPassword || "");
  if (newPassword.length < 8) {
    return NextResponse.json(
      { error: "PASSWORD_TOO_SHORT", minLength: 8 },
      { status: 400 }
    );
  }
  if (newPassword.length > 200) {
    return NextResponse.json({ error: "PASSWORD_TOO_LONG" }, { status: 400 });
  }

  const user = await prisma.user.findUnique({
    where: { id: session.user.id },
    select: { passwordHash: true },
  });
  if (!user) return NextResponse.json({ error: "NOT_FOUND" }, { status: 404 });

  if (user.passwordHash) {
    const current = String(body.currentPassword || "");
    if (!current) {
      return NextResponse.json(
        { error: "CURRENT_PASSWORD_REQUIRED" },
        { status: 400 }
      );
    }
    const ok = await bcrypt.compare(current, user.passwordHash);
    if (!ok) {
      return NextResponse.json({ error: "WRONG_CURRENT_PASSWORD" }, { status: 403 });
    }
  }

  const hash = await bcrypt.hash(newPassword, 10);
  await prisma.user.update({
    where: { id: session.user.id },
    data: { passwordHash: hash },
  });
  return NextResponse.json({ ok: true });
}

export async function DELETE() {
  const session = await auth();
  if (!session?.user?.id) {
    return NextResponse.json({ error: "UNAUTHORIZED" }, { status: 401 });
  }
  await prisma.user.update({
    where: { id: session.user.id },
    data: { passwordHash: null },
  });
  return NextResponse.json({ ok: true });
}
