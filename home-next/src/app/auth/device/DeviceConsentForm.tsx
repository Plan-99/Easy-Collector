"use client";

import { useState } from "react";

type Props = {
  nonce: string;
  machineId: string;
  hostname: string | null;
  os: string | null;
};

export default function DeviceConsentForm({ nonce, machineId, hostname, os }: Props) {
  const [busy, setBusy] = useState(false);
  const [done, setDone] = useState<"approved" | "rejected" | null>(null);
  const [error, setError] = useState<string | null>(null);

  async function submit(action: "approve" | "reject") {
    setBusy(true);
    setError(null);
    try {
      const res = await fetch("/api/device-auth/approve", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ nonce, action }),
      });
      const body = await res.json().catch(() => ({}));
      if (!res.ok) {
        if (body?.errorCode === "DEVICE_ALREADY_BOUND") {
          setError("이미 사용중인 기기가 있습니다. 대시보드에서 해제 후 다시 시도하세요.");
        } else {
          setError(body?.error || `요청 실패 (${res.status})`);
        }
        return;
      }
      setDone(action === "approve" ? "approved" : "rejected");
    } catch (e: any) {
      setError(e?.message || "네트워크 오류");
    } finally {
      setBusy(false);
    }
  }

  if (done === "approved") {
    return (
      <div className="rounded-xl border border-emerald-500/30 bg-emerald-500/5 p-5">
        <p className="font-semibold mb-2">기기가 등록되었습니다</p>
        <p className="text-surface-300 text-sm">
          런처가 자동으로 다음 단계로 넘어갑니다. 이 창은 닫으셔도 됩니다.
        </p>
      </div>
    );
  }

  if (done === "rejected") {
    return (
      <div className="rounded-xl border border-amber-500/30 bg-amber-500/5 p-5">
        <p className="font-semibold mb-2">요청이 거절되었습니다</p>
        <p className="text-surface-300 text-sm">런처에서 다시 시작할 수 있습니다.</p>
      </div>
    );
  }

  return (
    <div className="space-y-5">
      <div className="rounded-xl bg-surface-900/60 border border-white/5 p-5 space-y-3">
        <p className="text-surface-400 text-xs uppercase tracking-wider">기기 정보</p>
        <dl className="space-y-2 text-sm">
          <div className="flex gap-3">
            <dt className="text-surface-500 w-24 shrink-0">호스트명</dt>
            <dd className="text-surface-200">{hostname || "(미상)"}</dd>
          </div>
          <div className="flex gap-3">
            <dt className="text-surface-500 w-24 shrink-0">OS</dt>
            <dd className="text-surface-200">{os || "(미상)"}</dd>
          </div>
          <div className="flex gap-3">
            <dt className="text-surface-500 w-24 shrink-0">머신 ID</dt>
            <dd className="text-surface-200 font-mono text-xs break-all">{machineId}</dd>
          </div>
        </dl>
      </div>

      <p className="text-surface-300 text-sm">
        이 기기를 본인 계정에 등록하시겠습니까? 등록 후 한 계정당 1대의 기기만 활성 상태로 유지됩니다.
      </p>

      {error && (
        <div className="rounded-xl border border-red-500/30 bg-red-500/5 p-4 text-sm text-red-200">
          {error}
        </div>
      )}

      <div className="flex gap-3">
        <button
          type="button"
          disabled={busy}
          onClick={() => submit("approve")}
          className="flex-1 py-3 rounded-full bg-gradient-to-r from-indigo-500 to-purple-500 text-white font-semibold text-sm hover:opacity-90 disabled:opacity-50 spring-transition cursor-pointer"
        >
          {busy ? "처리 중…" : "이 기기 등록"}
        </button>
        <button
          type="button"
          disabled={busy}
          onClick={() => submit("reject")}
          className="flex-1 py-3 rounded-full bg-surface-800 text-surface-200 font-semibold text-sm hover:bg-surface-700 disabled:opacity-50 spring-transition cursor-pointer"
        >
          취소
        </button>
      </div>
    </div>
  );
}
