"use client";

import { useEffect } from "react";
import { useRouter } from "next/navigation";

// After mobile REDIRECTION-mode payment, PortOne lands the user back here
// with ?paymentId=…&code=…. Auto-call /api/orders/complete then strip query.
export default function OrderRedirectVerify() {
  const router = useRouter();
  useEffect(() => {
    if (typeof window === "undefined") return;
    const params = new URLSearchParams(window.location.search);
    const paymentId = params.get("paymentId");
    const code = params.get("code");
    if (!paymentId) return;
    history.replaceState(null, "", window.location.pathname);
    if (code) {
      // Treat as cancellation/failure — server-side state already reflects.
      router.refresh();
      return;
    }
    fetch("/api/orders/complete", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ paymentId }),
    })
      .catch(() => {})
      .finally(() => router.refresh());
  }, [router]);
  return null;
}
