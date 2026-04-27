"use client";

import { useState, useTransition } from "react";
import { useRouter } from "next/navigation";

export default function ResetBindingButton({
  serialKeyId,
  machineId,
}: {
  serialKeyId: string;
  machineId: string;
}) {
  const [isPending, startTransition] = useTransition();
  const [error, setError] = useState<string | null>(null);
  const router = useRouter();

  const handleReset = () => {
    if (
      !confirm(
        "이 라이선스 키의 기기 바인딩을 초기화하시겠습니까?\n사용자는 다시 다른 PC에서 활성화할 수 있게 됩니다."
      )
    ) {
      return;
    }

    setError(null);
    startTransition(async () => {
      try {
        const res = await fetch(
          `/api/admin/serial-key/${serialKeyId}/reset-binding`,
          { method: "POST" }
        );
        if (!res.ok) {
          const data = await res.json().catch(() => ({}));
          setError(data.error || "초기화 실패");
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
      <span
        className="text-xs text-emerald-400 font-mono"
        title={machineId}
      >
        {machineId.slice(0, 12)}...
      </span>
      <button
        type="button"
        onClick={handleReset}
        disabled={isPending}
        className="text-[10px] px-2 py-0.5 rounded bg-red-500/20 text-red-300 hover:bg-red-500/30 disabled:opacity-50 spring-transition"
      >
        {isPending ? "초기화 중..." : "리셋"}
      </button>
      {error && <span className="text-[10px] text-red-400">{error}</span>}
    </div>
  );
}
