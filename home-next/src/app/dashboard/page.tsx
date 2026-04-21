import { auth, signOut } from "@/auth";
import { prisma } from "@/lib/prisma";
import { redirect } from "next/navigation";
import Link from "next/link";

export default async function DashboardPage() {
  const session = await auth();
  if (!session?.user?.id) redirect("/auth/signin");

  const user = await prisma.user.findUnique({
    where: { id: session.user.id },
    include: { serialKeys: true },
  });

  if (!user) redirect("/auth/signin");

  const serialKey = user.serialKeys[0];

  return (
    <div className="min-h-dvh bg-surface-950 pt-16 px-6">
      <div className="max-w-3xl mx-auto py-16">
        {/* Header */}
        <div className="flex items-center justify-between mb-12">
          <div className="flex items-center gap-3">
            <div className="w-10 h-10 rounded-xl bg-gradient-to-br from-indigo-500 to-purple-500 flex items-center justify-center text-sm font-extrabold">
              ET
            </div>
            <span className="font-[var(--font-display)] font-bold text-xl tracking-tight">
              Dashboard
            </span>
          </div>
          <div className="flex items-center gap-4">
            <Link
              href="/"
              className="text-sm text-surface-400 hover:text-white spring-transition"
            >
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

        {/* Profile */}
        <div className="bezel-card mb-8">
          <div className="bezel-inner p-8">
            <div className="flex items-center gap-5 mb-6">
              {user.image ? (
                <img
                  src={user.image}
                  alt={user.name || ""}
                  className="w-14 h-14 rounded-full"
                />
              ) : (
                <div className="w-14 h-14 rounded-full bg-gradient-to-br from-indigo-500 to-purple-500 flex items-center justify-center font-bold text-lg">
                  {(user.name || user.email)?.[0]?.toUpperCase()}
                </div>
              )}
              <div>
                <h2 className="font-bold text-lg">{user.name}</h2>
                <p className="text-surface-400 text-sm">{user.email}</p>
              </div>
              <div className="ml-auto">
                <span
                  className={`px-3 py-1 rounded-full text-xs font-semibold ${
                    user.plan === "business"
                      ? "bg-indigo-500/15 text-indigo-300"
                      : "bg-surface-800 text-surface-400"
                  }`}
                >
                  {user.plan === "business" ? "Business" : "Free"} Plan
                </span>
              </div>
            </div>
          </div>
        </div>

        {/* License Key */}
        <div className="bezel-card mb-8">
          <div className="bezel-inner p-8">
            <h3 className="font-[var(--font-display)] font-bold text-lg mb-1">
              라이선스 키
            </h3>
            <p className="text-surface-500 text-sm mb-6">
              Easy Trainer 설치 후 이 키를 입력하세요. 1대의 PC에만 등록
              가능합니다.
            </p>

            {serialKey ? (
              <div className="space-y-4">
                <div className="flex items-center gap-4">
                  <code className="flex-1 px-5 py-3.5 rounded-xl bg-surface-950 border border-white/5 font-mono text-lg tracking-widest text-indigo-300">
                    {serialKey.key}
                  </code>
                  <CopyButton text={serialKey.key} />
                </div>
                <div className="flex items-center gap-6 text-sm text-surface-500">
                  <span>
                    상태:{" "}
                    <span
                      className={
                        serialKey.active ? "text-emerald-400" : "text-red-400"
                      }
                    >
                      {serialKey.active ? "활성" : "비활성"}
                    </span>
                  </span>
                  <span>
                    PC 바인딩:{" "}
                    {serialKey.machineId ? (
                      <span className="text-surface-300">
                        {serialKey.machineId.slice(0, 12)}...
                      </span>
                    ) : (
                      <span className="text-surface-500">미등록</span>
                    )}
                  </span>
                </div>
              </div>
            ) : (
              <p className="text-surface-500">
                라이선스 키가 아직 발급되지 않았습니다.
              </p>
            )}
          </div>
        </div>

        {/* Download */}
        <div className="bezel-card">
          <div className="bezel-inner p-8">
            <h3 className="font-[var(--font-display)] font-bold text-lg mb-1">
              다운로드
            </h3>
            <p className="text-surface-500 text-sm mb-6">
              Ubuntu 22.04 이상, NVIDIA GPU 필수
            </p>
            <a
              href="#"
              className="btn-pill bg-gradient-to-r from-indigo-500 to-purple-500 text-white shadow-lg shadow-indigo-500/20 hover:shadow-indigo-500/35 hover:-translate-y-0.5 text-sm"
            >
              Easy Trainer .deb 다운로드
              <span className="icon-circle bg-white/20">↓</span>
            </a>
          </div>
        </div>
      </div>
    </div>
  );
}

function CopyButton({ text }: { text: string }) {
  return <CopyButtonClient text={text} />;
}

import CopyButtonClient from "./CopyButtonClient";
