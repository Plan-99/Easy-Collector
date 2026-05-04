import { auth, signOut } from "@/auth";
import { redirect } from "next/navigation";
import Link from "next/link";
import Footer from "@/components/Footer";
import DashboardTabs from "./DashboardTabs";
import { getDashboardUser, isOnboarded } from "@/lib/dashboard-user";

export const dynamic = "force-dynamic";

export default async function DashboardLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  const session = await auth();
  if (!session?.user?.id) redirect("/auth/signin");

  // Cached: this fetch is shared with /dashboard/page.tsx via React cache()
  // so the layout + page combo runs a single Neon query per navigation.
  const me = await getDashboardUser(session.user.id);
  if (!isOnboarded(me)) redirect("/onboarding?next=/dashboard");

  return (
    <div className="min-h-dvh bg-surface-950 pt-16 flex flex-col">
      <div className="flex-1 px-6">
        <div className="max-w-3xl mx-auto py-12">
          <div className="flex items-center justify-between mb-8">
            <div className="flex items-center gap-3">
              <div className="w-10 h-10 rounded-xl bg-gradient-to-br from-indigo-500 to-purple-500 flex items-center justify-center text-sm font-extrabold">
                ET
              </div>
              <span className="font-[var(--font-display)] font-bold text-xl tracking-tight">
                내 정보
              </span>
            </div>
            <div className="flex items-center gap-4">
              <Link href="/" className="text-sm text-surface-400 hover:text-white spring-transition">
                홈
              </Link>
              <form
                action={async () => {
                  "use server";
                  await signOut({ redirectTo: "/" });
                }}
              >
                <button
                  type="submit"
                  className="text-sm text-surface-400 hover:text-white spring-transition"
                >
                  로그아웃
                </button>
              </form>
            </div>
          </div>
          <DashboardTabs />
          {children}
        </div>
      </div>
      <Footer />
    </div>
  );
}
