"use client";

import { useState, useTransition } from "react";
import { useRouter } from "next/navigation";

export default function UnbindDeviceButton({ deviceId }: { deviceId: string }) {
  const [isPending, startTransition] = useTransition();
  const [error, setError] = useState<string | null>(null);
  const router = useRouter();

  const handleUnbind = () => {
    if (
      !confirm(
        "이 기기의 등록을 해제하시겠습니까?\n런처는 다시 Google 로그인을 요구합니다."
      )
    )
      return;
    setError(null);
    startTransition(async () => {
      try {
        const res = await fetch(`/api/devices/${deviceId}/unbind`, { method: "POST" });
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
    <div className="flex flex-col items-end gap-1">
      <button
        type="button"
        onClick={handleUnbind}
        disabled={isPending}
        className="text-xs px-3 py-1.5 rounded-full bg-red-500/15 text-red-300 hover:bg-red-500/25 disabled:opacity-50 spring-transition cursor-pointer"
      >
        {isPending ? "해제 중…" : "등록 해제"}
      </button>
      {error && <span className="text-[10px] text-red-400">{error}</span>}
    </div>
  );
}
