"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";

export default function CancelOrderButton({ orderId }: { orderId: string }) {
  const router = useRouter();
  const [pending, setPending] = useState(false);

  async function cancel() {
    const reason = prompt("취소 사유를 입력하세요:", "관리자 취소");
    if (!reason) return;
    setPending(true);
    try {
      const res = await fetch(`/api/admin/orders/${orderId}/cancel`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ reason }),
      });
      if (!res.ok) {
        const b = await res.json().catch(() => ({}));
        throw new Error(b?.error || "취소 실패");
      }
      router.refresh();
    } catch (err) {
      alert(err instanceof Error ? err.message : "알 수 없는 오류");
      setPending(false);
    }
  }

  return (
    <button
      type="button"
      onClick={cancel}
      disabled={pending}
      className="px-5 py-2 rounded-full bg-red-500/20 hover:bg-red-500/30 border border-red-500/30 text-red-200 text-xs font-semibold disabled:opacity-50"
    >
      {pending ? "처리 중…" : "주문 취소 + 환불"}
    </button>
  );
}
