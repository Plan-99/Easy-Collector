"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";

const ERROR_LABEL: Record<string, string> = {
  NOT_PENDING: "이미 처리된 주문입니다.",
  ALREADY_PAID:
    "결제가 완료되어 취소할 수 없습니다. 잠시 후 주문 상세에서 결제 완료 상태로 표시됩니다.",
  NOT_FOUND: "주문을 찾을 수 없습니다.",
  UNAUTHORIZED: "로그인이 필요합니다.",
};

export default function CancelPendingButton({
  orderId,
  variant = "primary",
}: {
  orderId: string;
  variant?: "primary" | "inline";
}) {
  const router = useRouter();
  const [busy, setBusy] = useState(false);

  async function handleClick(e: React.MouseEvent) {
    e.preventDefault();
    e.stopPropagation();
    if (!confirm("이 결제 대기 주문을 취소하시겠습니까?")) return;
    setBusy(true);
    try {
      const res = await fetch(`/api/orders/${orderId}/cancel`, { method: "POST" });
      if (!res.ok) {
        const b = (await res.json().catch(() => ({}))) as { error?: string };
        const msg = (b.error && ERROR_LABEL[b.error]) || b.error || "취소 실패";
        alert(msg);
        // Refresh anyway — server state may have changed (e.g. ALREADY_PAID)
        router.refresh();
        return;
      }
      router.refresh();
    } finally {
      setBusy(false);
    }
  }

  if (variant === "inline") {
    return (
      <button
        type="button"
        onClick={handleClick}
        disabled={busy}
        className="text-xs text-red-300 hover:text-red-200 underline disabled:opacity-50 cursor-pointer"
      >
        {busy ? "취소 중…" : "주문 취소"}
      </button>
    );
  }
  return (
    <button
      type="button"
      onClick={handleClick}
      disabled={busy}
      className="px-4 py-2 rounded-full bg-red-500/15 hover:bg-red-500/25 border border-red-500/30 text-red-200 text-xs font-semibold disabled:opacity-50 cursor-pointer"
    >
      {busy ? "취소 중…" : "주문 취소"}
    </button>
  );
}
