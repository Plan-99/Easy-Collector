"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";

export default function RefundButton({ paymentId }: { paymentId: string }) {
  const router = useRouter();
  const [pending, setPending] = useState(false);

  async function handleRefund() {
    if (!confirm("이 결제를 환불하시겠습니까? 보유 모듈 권한이 회수됩니다.")) return;
    setPending(true);
    try {
      const res = await fetch(`/api/payments/${encodeURIComponent(paymentId)}/refund`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ reason: "사용자 요청 (마이페이지)" }),
      });
      if (!res.ok) {
        const body = await res.json().catch(() => ({}));
        throw new Error(body?.error || "환불 처리에 실패했습니다.");
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
      onClick={handleRefund}
      disabled={pending}
      className="text-xs text-red-300 hover:text-red-200 underline disabled:opacity-50 cursor-pointer disabled:cursor-wait"
    >
      {pending ? "처리 중…" : "환불"}
    </button>
  );
}
