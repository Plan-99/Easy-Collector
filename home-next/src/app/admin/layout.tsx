import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";
import { redirect } from "next/navigation";
import Link from "next/link";
import Footer from "@/components/Footer";
import AdminTabs from "./AdminTabs";

export const dynamic = "force-dynamic";

export default async function AdminLayout({ children }: { children: React.ReactNode }) {
  const session = await auth();
  if (!session?.user?.id) redirect("/auth/signin");

  const me = await prisma.user.findUnique({ where: { id: session.user.id } });
  if (me?.role !== "admin") redirect("/dashboard");

  const onboarded =
    !!me.organization &&
    !!me.department &&
    !!me.jobRole &&
    !!me.termsAcceptedAt &&
    !!me.privacyAcceptedAt &&
    !!me.refundAcceptedAt;
  if (!onboarded) redirect("/onboarding?next=/admin");

  return (
    <div className="min-h-dvh bg-surface-950 flex flex-col">
      <div className="flex-1 px-6 py-20">
        <div className="max-w-6xl mx-auto">
          <div className="flex items-center justify-between mb-8">
            <h1 className="font-[var(--font-display)] font-bold text-2xl">
              관리자 대시보드
            </h1>
            <Link
              href="/dashboard"
              className="text-sm text-surface-400 hover:text-white spring-transition"
            >
              ← 내 정보
            </Link>
          </div>
          <AdminTabs />
          {children}
        </div>
      </div>
      <Footer />
    </div>
  );
}
