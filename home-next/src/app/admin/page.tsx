import { auth } from "@/auth";
import { prisma } from "@/lib/prisma";
import { redirect } from "next/navigation";
import Link from "next/link";

export default async function AdminPage() {
  const session = await auth();

  if (!session?.user) {
    redirect("/auth/signin");
  }

  // Check admin role
  const user = await prisma.user.findUnique({
    where: { id: session.user.id },
  });

  if (user?.role !== "admin") {
    redirect("/dashboard");
  }

  // Fetch all users with their serial keys
  const users = await prisma.user.findMany({
    include: {
      serialKeys: true,
    },
    orderBy: { createdAt: "desc" },
  });

  return (
    <div className="min-h-dvh bg-surface-950 px-6 py-20">
      <div className="max-w-6xl mx-auto">
        <div className="flex items-center justify-between mb-8">
          <div>
            <h1 className="font-[var(--font-display)] font-bold text-2xl">
              관리자 대시보드
            </h1>
            <p className="text-surface-400 text-sm mt-1">
              총 {users.length}명의 사용자
            </p>
          </div>
          <Link
            href="/dashboard"
            className="text-sm text-surface-400 hover:text-white spring-transition"
          >
            ← 내 정보
          </Link>
        </div>

        <div className="bezel-card">
          <div className="bezel-inner p-0 overflow-hidden">
            <table className="w-full text-sm">
              <thead>
                <tr className="border-b border-white/10 text-left text-surface-400">
                  <th className="px-5 py-3 font-medium">사용자</th>
                  <th className="px-5 py-3 font-medium">이메일</th>
                  <th className="px-5 py-3 font-medium">플랜</th>
                  <th className="px-5 py-3 font-medium">라이선스 키</th>
                  <th className="px-5 py-3 font-medium">기기 바인딩</th>
                  <th className="px-5 py-3 font-medium">최근 접속</th>
                  <th className="px-5 py-3 font-medium">가입일</th>
                </tr>
              </thead>
              <tbody>
                {users.map((u) => {
                  const sk = u.serialKeys[0];
                  return (
                    <tr
                      key={u.id}
                      className="border-b border-white/5 hover:bg-white/[0.02] spring-transition"
                    >
                      <td className="px-5 py-3">
                        <div className="flex items-center gap-2">
                          {u.image && (
                            <img
                              src={u.image}
                              alt=""
                              className="w-6 h-6 rounded-full"
                            />
                          )}
                          <span className="font-medium">
                            {u.name || "이름 없음"}
                          </span>
                          {u.role === "admin" && (
                            <span className="text-[10px] px-1.5 py-0.5 rounded bg-indigo-500/20 text-indigo-300 font-semibold">
                              관리자
                            </span>
                          )}
                        </div>
                      </td>
                      <td className="px-5 py-3 text-surface-400">{u.email}</td>
                      <td className="px-5 py-3">
                        <span
                          className={`text-xs font-semibold px-2 py-0.5 rounded ${
                            (sk?.plan || u.plan) === "business"
                              ? "bg-amber-500/20 text-amber-300"
                              : "bg-surface-800 text-surface-300"
                          }`}
                        >
                          {(sk?.plan || u.plan).toUpperCase()}
                        </span>
                      </td>
                      <td className="px-5 py-3 font-mono text-xs text-surface-400">
                        {sk?.key || "-"}
                      </td>
                      <td className="px-5 py-3">
                        {sk?.machineId ? (
                          <span className="text-xs text-emerald-400 font-mono">
                            {sk.machineId.slice(0, 12)}...
                          </span>
                        ) : (
                          <span className="text-xs text-surface-600">
                            미연결
                          </span>
                        )}
                      </td>
                      <td className="px-5 py-3 text-surface-400 text-xs">
                        {sk?.lastActiveAt
                          ? new Date(sk.lastActiveAt).toLocaleString("ko-KR")
                          : "-"}
                      </td>
                      <td className="px-5 py-3 text-surface-400 text-xs">
                        {new Date(u.createdAt).toLocaleDateString("ko-KR")}
                      </td>
                    </tr>
                  );
                })}
              </tbody>
            </table>
          </div>
        </div>
      </div>
    </div>
  );
}
