"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";

type Initial = {
  id: string;
  name: string;
  category: string;
  priceKrw: number;
  active: boolean;
};

export default function ModuleRow({ initial }: { initial: Initial }) {
  const router = useRouter();
  const [name, setName] = useState(initial.name);
  const [priceKrw, setPriceKrw] = useState(initial.priceKrw);
  const [active, setActive] = useState(initial.active);
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const dirty =
    name.trim() !== initial.name.trim() ||
    priceKrw !== initial.priceKrw ||
    active !== initial.active;

  async function save() {
    setSaving(true);
    setError(null);
    try {
      const res = await fetch(`/api/admin/modules/${encodeURIComponent(initial.id)}`, {
        method: "PATCH",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          name: name.trim(),
          priceKrw,
          active,
        }),
      });
      if (!res.ok) {
        const body = await res.json().catch(() => ({}));
        throw new Error(body?.error || "저장 실패");
      }
      router.refresh();
    } catch (err) {
      setError(err instanceof Error ? err.message : "알 수 없는 오류");
    } finally {
      setSaving(false);
    }
  }

  return (
    <tr className="border-b border-white/5 hover:bg-white/[0.02] spring-transition">
      <td className="px-5 py-3">
        <code className="text-xs text-surface-400">{initial.id}</code>
      </td>
      <td className="px-5 py-3">
        <input
          type="text"
          value={name}
          onChange={e => setName(e.target.value)}
          className="w-full px-2.5 py-1.5 rounded bg-surface-900/60 border border-white/10 text-surface-100 text-sm focus:outline-none focus:border-indigo-500/60 focus:ring-1 focus:ring-indigo-500/30"
        />
      </td>
      <td className="px-5 py-3 text-surface-400 text-xs">{initial.category}</td>
      <td className="px-5 py-3 text-right">
        <input
          type="number"
          min={0}
          step={1000}
          value={priceKrw}
          onChange={e => {
            const v = Number(e.target.value);
            setPriceKrw(Number.isFinite(v) && v >= 0 ? Math.floor(v) : 0);
          }}
          className="w-32 px-2.5 py-1.5 text-right rounded bg-surface-900/60 border border-white/10 text-surface-100 text-sm font-mono focus:outline-none focus:border-indigo-500/60 focus:ring-1 focus:ring-indigo-500/30"
        />
      </td>
      <td className="px-5 py-3 text-center">
        <input
          type="checkbox"
          checked={active}
          onChange={e => setActive(e.target.checked)}
          className="w-4 h-4 accent-indigo-500"
        />
      </td>
      <td className="px-5 py-3 text-right">
        <div className="flex flex-col items-end gap-1">
          <button
            type="button"
            onClick={save}
            disabled={!dirty || saving}
            className="px-3 py-1.5 rounded text-xs font-semibold bg-indigo-500/20 hover:bg-indigo-500/30 border border-indigo-500/30 text-indigo-200 disabled:opacity-30 disabled:cursor-not-allowed cursor-pointer spring-transition"
          >
            {saving ? "저장 중…" : "저장"}
          </button>
          {error && <span className="text-[10px] text-red-400">{error}</span>}
        </div>
      </td>
    </tr>
  );
}
