"use client";

import { useState, useTransition } from "react";
import { useRouter } from "next/navigation";

export default function UnbindDeviceButton({
  deviceId,
  label,
}: {
  deviceId: string;
  label: string;
}) {
  const [isPending, startTransition] = useTransition();
  const [error, setError] = useState<string | null>(null);
  const router = useRouter();

  const handleUnbind = () => {
    if (
      !confirm(
        "이 사용자의 기기 등록을 해제하시겠습니까?\n다음 런처 실행 시 다시 Google 로그인이 필요합니다."
      )
    )
      return;
    setError(null);
    startTransition(async () => {
      try {
        const res = await fetch(`/api/admin/devices/${deviceId}/unbind`, {
          method: "POST",
        });
        if (!res.ok) {
          const data = await res.json().catch(() => ({}));
          setError(data.error || "해제 실패");
          return;
        }
        router.refresh();
      } catch (e) {
        setError(String(e));
      }
    });
  };

  return (
    <div className="flex items-center gap-2">
      <span className="text-xs text-emerald-400 font-mono" title={deviceId}>
        {label}
      </span>
      <button
        type="button"
        onClick={handleUnbind}
        disabled={isPending}
        className="text-[10px] px-2 py-0.5 rounded bg-red-500/20 text-red-300 hover:bg-red-500/30 disabled:opacity-50 spring-transition cursor-pointer"
      >
        {isPending ? "해제 중…" : "리셋"}
      </button>
      {error && <span className="text-[10px] text-red-400">{error}</span>}
    </div>
  );
}
