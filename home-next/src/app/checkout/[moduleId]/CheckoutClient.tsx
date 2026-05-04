"use client";

import { useEffect, useState } from "react";
import * as PortOne from "@portone/browser-sdk/v2";

type Props = {
  moduleId: string;
  moduleName: string;
  priceKrw: number;
  buyerEmail: string;
  buyerName: string;
  customerId: string; // truncated to ≤20 chars by server (PG limit)
  storeId: string;
  channelKey: string; // PG channel — handles all payment methods inside its UI
};

type Phase = "idle" | "initiating" | "paying" | "verifying" | "done" | "error";

export default function CheckoutClient(props: Props) {
  const [phase, setPhase] = useState<Phase>("idle");
  const [error, setError] = useState<string | null>(null);

  // After a REDIRECTION-mode payment we land back here with ?paymentId=…&code=….
  // Auto-verify on mount so the user sees the success screen without clicking
  // again. Strip the query so a refresh doesn't re-trigger the flow.
  useEffect(() => {
    if (typeof window === "undefined") return;
    const params = new URLSearchParams(window.location.search);
    const returnedPaymentId = params.get("paymentId");
    const code = params.get("code");
    const message = params.get("message");
    if (!returnedPaymentId) return;

    history.replaceState(null, "", window.location.pathname);

    if (code) {
      setPhase("error");
      setError(message || "결제가 취소되었거나 실패했습니다.");
      return;
    }

    setPhase("verifying");
    fetch("/api/payments/complete", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ paymentId: returnedPaymentId }),
    })
      .then(async res => {
        if (!res.ok) {
          const body = await res.json().catch(() => ({}));
          throw new Error(body?.error || "결제 검증에 실패했습니다.");
        }
        setPhase("done");
      })
      .catch(err => {
        setPhase("error");
        setError(err instanceof Error ? err.message : "알 수 없는 오류");
      });
  }, []);

  async function handlePay() {
    setError(null);
    setPhase("initiating");
    try {
      const initRes = await fetch("/api/payments/checkout", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ moduleId: props.moduleId }),
      });
      if (!initRes.ok) {
        const body = await initRes.json().catch(() => ({}));
        throw new Error(body?.error || "결제 준비에 실패했습니다.");
      }
      const { paymentId, amountKrw, orderName } = (await initRes.json()) as {
        paymentId: string;
        amountKrw: number;
        orderName: string;
      };

      setPhase("paying");
      // Single PG channel (한국결제네트웍스) — its UI lists card + 카카오페이/
      // 네이버페이/토스페이 as tabs, so we just call CARD and let the user
      // pick inside.
      const paymentRequest = {
        storeId: props.storeId,
        channelKey: props.channelKey,
        paymentId,
        orderName,
        totalAmount: amountKrw,
        currency: "CURRENCY_KRW",
        payMethod: "CARD",
        customer: {
          customerId: props.customerId,
          email: props.buyerEmail || undefined,
          fullName: props.buyerName || undefined,
        },
        customData: { moduleId: props.moduleId },
        windowType: { mobile: "REDIRECTION" },
        redirectUrl:
          typeof window !== "undefined"
            ? `${window.location.origin}/checkout/${encodeURIComponent(props.moduleId)}?paymentId=${encodeURIComponent(paymentId)}`
            : undefined,
      };
      const result = await PortOne.requestPayment(
        paymentRequest as unknown as Parameters<typeof PortOne.requestPayment>[0]
      );

      if (!result || result.code != null) {
        throw new Error(result?.message || "결제가 취소되었거나 실패했습니다.");
      }

      setPhase("verifying");
      const completeRes = await fetch("/api/payments/complete", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ paymentId }),
      });
      if (!completeRes.ok) {
        const body = await completeRes.json().catch(() => ({}));
        throw new Error(body?.error || "결제 검증에 실패했습니다.");
      }

      setPhase("done");
    } catch (err) {
      setPhase("error");
      setError(err instanceof Error ? err.message : "알 수 없는 오류");
    }
  }

  if (phase === "done") {
    return (
      <div className="rounded-xl border border-emerald-500/30 bg-emerald-500/5 p-6 space-y-4">
        <p className="font-bold text-emerald-300 text-lg">결제가 완료되었습니다 ✓</p>
        <p className="text-surface-300 text-sm leading-relaxed">
          {props.moduleName} 모듈이 계정에 추가되었습니다. 이 창을 닫고 런처로 돌아가
          모듈을 설치할 수 있습니다.
        </p>
        <div className="flex gap-3">
          <a
            href="/dashboard"
            className="flex-1 text-center py-3 rounded-full bg-white/10 hover:bg-white/15 border border-white/10 text-sm font-semibold spring-transition"
          >
            대시보드 열기
          </a>
          <button
            type="button"
            onClick={() => window.close()}
            className="flex-1 py-3 rounded-full bg-gradient-to-r from-indigo-500 to-purple-500 text-white text-sm font-semibold spring-transition cursor-pointer"
          >
            창 닫기
          </button>
        </div>
      </div>
    );
  }

  const busy = phase === "initiating" || phase === "paying" || phase === "verifying";

  return (
    <div className="space-y-4">
      {error && (
        <p className="text-sm text-red-400 bg-red-500/10 border border-red-500/20 rounded-lg px-3 py-2">
          {error}
        </p>
      )}
      <button
        type="button"
        onClick={handlePay}
        disabled={busy}
        className="w-full py-3.5 rounded-full bg-gradient-to-r from-indigo-500 to-purple-500 text-white font-semibold text-sm shadow-lg shadow-indigo-500/25 hover:shadow-indigo-500/40 spring-transition disabled:opacity-50 disabled:cursor-wait cursor-pointer"
      >
        {phase === "idle" && "결제하기"}
        {phase === "initiating" && "결제 준비 중…"}
        {phase === "paying" && "결제 진행 중…"}
        {phase === "verifying" && "결제 확인 중…"}
        {phase === "error" && "다시 시도"}
      </button>
      <p className="text-xs text-surface-500 text-center">
        결제창에서 카드·카카오페이·네이버페이 등 원하는 수단을 선택할 수 있습니다.
      </p>
    </div>
  );
}
