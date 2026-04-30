import { prisma } from "@/lib/prisma";
import UnbindDeviceButton from "../UnbindDeviceButton";

export default async function AdminUsersPage() {
  const users = await prisma.user.findMany({
    include: {
      devices: { where: { active: true }, orderBy: { lastSeenAt: "desc" } },
      _count: { select: { entitlements: true, payments: true } },
    },
    orderBy: { createdAt: "desc" },
  });

  return (
    <>
      <p className="text-surface-400 text-sm mb-4">총 {users.length}명의 사용자</p>
      <div className="bezel-card">
        <div className="bezel-inner p-0 overflow-hidden">
          <table className="w-full text-sm">
            <thead>
              <tr className="border-b border-white/10 text-left text-surface-400">
                <th className="px-5 py-3 font-medium">사용자</th>
                <th className="px-5 py-3 font-medium">이메일</th>
                <th className="px-5 py-3 font-medium">플랜</th>
                <th className="px-5 py-3 font-medium">활성 기기</th>
                <th className="px-5 py-3 font-medium">보유 모듈</th>
                <th className="px-5 py-3 font-medium">결제</th>
                <th className="px-5 py-3 font-medium">최근 접속</th>
                <th className="px-5 py-3 font-medium">가입일</th>
              </tr>
            </thead>
            <tbody>
              {users.map(u => {
                const device = u.devices[0];
                return (
                  <tr
                    key={u.id}
                    className="border-b border-white/5 hover:bg-white/[0.02] spring-transition"
                  >
                    <td className="px-5 py-3">
                      <div className="flex items-center gap-2">
                        {u.image && (
                          // eslint-disable-next-line @next/next/no-img-element
                          <img src={u.image} alt="" className="w-6 h-6 rounded-full" />
                        )}
                        <span className="font-medium">{u.name || "이름 없음"}</span>
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
                          u.plan === "business"
                            ? "bg-amber-500/20 text-amber-300"
                            : u.plan === "unlimited"
                              ? "bg-indigo-500/20 text-indigo-200"
                              : "bg-surface-800 text-surface-300"
                        }`}
                      >
                        {u.plan.toUpperCase()}
                      </span>
                    </td>
                    <td className="px-5 py-3">
                      {device ? (
                        <UnbindDeviceButton
                          deviceId={device.id}
                          label={device.hostname || device.machineId.slice(0, 12) + "…"}
                        />
                      ) : (
                        <span className="text-xs text-surface-600">미등록</span>
                      )}
                    </td>
                    <td className="px-5 py-3 text-surface-300 text-xs">{u._count.entitlements}</td>
                    <td className="px-5 py-3 text-surface-300 text-xs">{u._count.payments}</td>
                    <td className="px-5 py-3 text-surface-400 text-xs">
                      {device?.lastSeenAt
                        ? new Date(device.lastSeenAt).toLocaleString("ko-KR")
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
    </>
  );
}
