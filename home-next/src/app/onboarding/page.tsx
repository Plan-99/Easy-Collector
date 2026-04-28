import { auth, signOut } from "@/auth";
import { prisma } from "@/lib/prisma";
import { redirect } from "next/navigation";
import Footer from "@/components/Footer";
import OnboardingForm from "./OnboardingForm";

export const metadata = {
  title: "가입 정보 입력 — Easy Trainer",
};

export default async function OnboardingPage({
  searchParams,
}: {
  searchParams: Promise<{ next?: string }>;
}) {
  const session = await auth();
  if (!session?.user?.id) redirect("/auth/signin");

  const user = await prisma.user.findUnique({
    where: { id: session.user.id },
    select: {
      name: true,
      email: true,
      image: true,
      organization: true,
      department: true,
      jobRole: true,
      termsAcceptedAt: true,
      privacyAcceptedAt: true,
      refundAcceptedAt: true,
    },
  });
  if (!user) redirect("/auth/signin");

  const params = await searchParams;
  const next = typeof params.next === "string" && params.next.startsWith("/") ? params.next : "/dashboard";

  // Already onboarded → straight through.
  const onboarded =
    !!user.organization &&
    !!user.department &&
    !!user.jobRole &&
    !!user.termsAcceptedAt &&
    !!user.privacyAcceptedAt &&
    !!user.refundAcceptedAt;
  if (onboarded) redirect(next);

  return (
    <div className="min-h-dvh flex flex-col bg-surface-950">
      <div className="flex-1 px-6 py-16 flex items-center justify-center">
        <div className="w-full max-w-xl">
          <div className="flex items-center gap-2.5 mb-8">
            <div className="w-10 h-10 rounded-xl bg-gradient-to-br from-indigo-500 to-purple-500 flex items-center justify-center text-sm font-extrabold">
              ET
            </div>
            <span className="font-[var(--font-display)] font-bold text-xl tracking-tight">
              Easy Trainer
            </span>
          </div>

          <h1 className="font-[var(--font-display)] font-extrabold text-3xl mb-3">
            가입을 완료해 주세요
          </h1>
          <p className="text-surface-400 text-sm mb-10 leading-relaxed">
            결제·세금계산서 발행 및 서비스 운영을 위해 소속 정보를 입력 받습니다.
            입력한 정보는 마이페이지에서 언제든지 수정할 수 있습니다.
          </p>

          <div className="bezel-card mb-6">
            <div className="bezel-inner p-6 flex items-center gap-4">
              {user.image ? (
                // eslint-disable-next-line @next/next/no-img-element
                <img src={user.image} alt={user.name || ""} className="w-12 h-12 rounded-full" />
              ) : (
                <div className="w-12 h-12 rounded-full bg-gradient-to-br from-indigo-500 to-purple-500 flex items-center justify-center font-bold">
                  {(user.name || user.email)?.[0]?.toUpperCase()}
                </div>
              )}
              <div className="min-w-0 flex-1">
                <p className="font-semibold truncate">{user.name || "(이름 없음)"}</p>
                <p className="text-surface-400 text-sm truncate">{user.email}</p>
              </div>
              <form
                action={async () => {
                  "use server";
                  await signOut({ redirectTo: "/" });
                }}
              >
                <button
                  type="submit"
                  className="text-xs text-surface-500 hover:text-surface-200 spring-transition"
                >
                  다른 계정으로 로그인
                </button>
              </form>
            </div>
          </div>

          <OnboardingForm
            initial={{
              organization: user.organization || "",
              department: user.department || "",
              jobRole: user.jobRole || "",
            }}
            nextUrl={next}
          />
        </div>
      </div>
      <Footer />
    </div>
  );
}
