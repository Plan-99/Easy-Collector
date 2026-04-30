import { NextResponse } from "next/server";
import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";

export async function POST(req: Request) {
  const session = await auth();
  if (!session?.user?.id) {
    return NextResponse.json({ error: "UNAUTHORIZED" }, { status: 401 });
  }

  let body: unknown;
  try {
    body = await req.json();
  } catch {
    return NextResponse.json({ error: "INVALID_JSON" }, { status: 400 });
  }

  const data = body as Partial<{
    organization: string;
    department: string;
    jobRole: string;
    agreedTerms: boolean;
    agreedPrivacy: boolean;
    agreedRefund: boolean;
  }>;

  const organization = (data.organization || "").trim().slice(0, 200);
  const department = (data.department || "").trim().slice(0, 200);
  const jobRole = (data.jobRole || "").trim().slice(0, 200);

  if (!organization || !department || !jobRole) {
    return NextResponse.json({ error: "MISSING_FIELDS" }, { status: 400 });
  }
  if (!data.agreedTerms || !data.agreedPrivacy || !data.agreedRefund) {
    return NextResponse.json({ error: "AGREEMENTS_REQUIRED" }, { status: 400 });
  }

  const now = new Date();
  await prisma.user.update({
    where: { id: session.user.id },
    data: {
      organization,
      department,
      jobRole,
      termsAcceptedAt: now,
      privacyAcceptedAt: now,
      refundAcceptedAt: now,
    },
  });

  return NextResponse.json({ ok: true });
}
