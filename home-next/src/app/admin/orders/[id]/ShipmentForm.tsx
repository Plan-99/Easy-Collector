"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";

const COURIERS = [
  "CJ대한통운",
  "한진택배",
  "롯데택배",
  "우체국택배",
  "로젠택배",
  "쿠팡로지스틱스",
  "기타",
];

export default function ShipmentForm({
  orderId,
  initial,
  orderStatus,
}: {
  orderId: string;
  initial: { courier: string; trackingNumber: string; shippedAt: string } | null;
  orderStatus: string;
}) {
  const router = useRouter();
  const [courier, setCourier] = useState(initial?.courier || "CJ대한통운");
  const [trackingNumber, setTrackingNumber] = useState(initial?.trackingNumber || "");
  const [shippedAt, setShippedAt] = useState(
    initial?.shippedAt || new Date().toISOString().slice(0, 10)
  );
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const canEdit = orderStatus === "PAID" || orderStatus === "SHIPPED";

  async function save() {
    if (!trackingNumber.trim()) {
      setError("송장번호를 입력하세요.");
      return;
    }
    setSaving(true);
    setError(null);
    try {
      const res = await fetch(`/api/admin/orders/${orderId}/shipment`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          courier,
          trackingNumber: trackingNumber.trim(),
          shippedAt: shippedAt || null,
        }),
      });
      if (!res.ok) {
        const b = await res.json().catch(() => ({}));
        throw new Error(b?.error || "저장 실패");
      }
      router.refresh();
    } catch (err) {
      setError(err instanceof Error ? err.message : "알 수 없는 오류");
    } finally {
      setSaving(false);
    }
  }

  async function markDelivered() {
    if (!confirm("배송 완료 상태로 변경하시겠습니까?")) return;
    setSaving(true);
    try {
      const res = await fetch(`/api/admin/orders/${orderId}/shipment`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ markDelivered: true }),
      });
      if (!res.ok) throw new Error("실패");
      router.refresh();
    } catch (err) {
      alert(err instanceof Error ? err.message : "알 수 없는 오류");
      setSaving(false);
    }
  }

  if (!canEdit && !initial) {
    return (
      <p className="text-sm text-surface-500">
        주문이 결제 완료(PAID) 상태일 때 송장을 등록할 수 있습니다. 현재 상태: {orderStatus}
      </p>
    );
  }

  return (
    <div className="space-y-3">
      <div className="grid grid-cols-3 gap-3">
        <label className="block">
          <span className="text-xs font-semibold text-surface-300">택배사</span>
          <select
            value={courier}
            onChange={e => setCourier(e.target.value)}
            disabled={!canEdit}
            className="mt-1 w-full px-3 py-2 rounded-lg bg-surface-900/60 border border-white/10 text-surface-100 text-sm focus:outline-none focus:border-indigo-500/60 disabled:opacity-60"
          >
            {COURIERS.map(c => (
              <option key={c} value={c}>{c}</option>
            ))}
          </select>
        </label>
        <label className="block">
          <span className="text-xs font-semibold text-surface-300">송장번호</span>
          <input
            type="text"
            value={trackingNumber}
            onChange={e => setTrackingNumber(e.target.value)}
            disabled={!canEdit}
            placeholder="123456789012"
            className="mt-1 w-full px-3 py-2 rounded-lg bg-surface-900/60 border border-white/10 text-surface-100 text-sm font-mono focus:outline-none focus:border-indigo-500/60 disabled:opacity-60"
          />
        </label>
        <label className="block">
          <span className="text-xs font-semibold text-surface-300">출고일</span>
          <input
            type="date"
            value={shippedAt}
            onChange={e => setShippedAt(e.target.value)}
            disabled={!canEdit}
            className="mt-1 w-full px-3 py-2 rounded-lg bg-surface-900/60 border border-white/10 text-surface-100 text-sm focus:outline-none focus:border-indigo-500/60 disabled:opacity-60"
          />
        </label>
      </div>
      {error && (
        <p className="text-xs text-red-400 bg-red-500/10 border border-red-500/20 rounded-md px-3 py-2">
          {error}
        </p>
      )}
      <div className="flex gap-2">
        {canEdit && (
          <button
            type="button"
            onClick={save}
            disabled={saving || !trackingNumber.trim()}
            className="px-5 py-2 rounded-full bg-indigo-500/20 hover:bg-indigo-500/30 border border-indigo-500/30 text-indigo-200 text-xs font-semibold spring-transition disabled:opacity-50"
          >
            {saving ? "저장 중…" : initial ? "송장 수정" : "송장 등록 (출고 처리)"}
          </button>
        )}
        {orderStatus === "SHIPPED" && (
          <button
            type="button"
            onClick={markDelivered}
            disabled={saving}
            className="px-5 py-2 rounded-full bg-emerald-500/20 hover:bg-emerald-500/30 border border-emerald-500/30 text-emerald-200 text-xs font-semibold disabled:opacity-50"
          >
            배송 완료 처리
          </button>
        )}
      </div>
    </div>
  );
}
