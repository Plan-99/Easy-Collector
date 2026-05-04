"use client";

import { useEffect, useMemo, useRef, useState } from "react";
import Link from "next/link";

type Module = {
  id: string;
  name: string;
  category: string;
  description: string | null;
  priceKrw: number;
};

function formatKrw(amountKrw: number) {
  return new Intl.NumberFormat("ko-KR", {
    style: "currency",
    currency: "KRW",
    maximumFractionDigits: 0,
  }).format(amountKrw);
}

// Static module-id → image mapping. Files live under home-next/public/modules/.
// Modules without a specific image fall back to a category placeholder.
const MODULE_IMAGE: Record<string, string> = {
  robot_piper: "/modules/piper.png",
  robot_unitree: "/modules/unitree.png",
  robot_rbpodo: "/modules/rainbow.png",
  robot_kinova: "/modules/kinova.png",
  gripper_robotiq: "/modules/robotiq.png",
  gripper_onrobot: "/modules/onrobot.png",
  gripper_generic: "/modules/custom.png",
  custom_interfaces: "/modules/custom.png",
  sensor_webcam: "/modules/webcam.png",
  sensor_realsense: "/modules/realsense.png",
};

function moduleImage(m: Module): string | null {
  return MODULE_IMAGE[m.id] || null;
}

export default function ModuleCatalog({
  modules,
  ownedModuleIds,
  categoryLabels,
  visibleCategories,
  focusedId,
}: {
  modules: Module[];
  ownedModuleIds: string[];
  categoryLabels: Record<string, string>;
  visibleCategories: string[];
  focusedId: string;
}) {
  const [query, setQuery] = useState("");
  const focusRef = useRef<HTMLDivElement | null>(null);
  const ownedSet = useMemo(() => new Set(ownedModuleIds), [ownedModuleIds]);

  useEffect(() => {
    if (focusedId && focusRef.current) {
      focusRef.current.scrollIntoView({ behavior: "smooth", block: "center" });
    }
  }, [focusedId]);

  const grouped = useMemo(() => {
    const filter = query.trim().toLowerCase();
    const out: Record<string, Module[]> = {};
    for (const m of modules) {
      if (filter) {
        const hay = `${m.name} ${m.description || ""} ${m.id}`.toLowerCase();
        if (!hay.includes(filter)) continue;
      }
      if (!out[m.category]) out[m.category] = [];
      out[m.category].push(m);
    }
    return out;
  }, [modules, query]);

  const totalShown = Object.values(grouped).reduce((sum, arr) => sum + arr.length, 0);

  return (
    <div className="space-y-8">
      <div>
        <h2 className="font-[var(--font-display)] font-bold text-xl mb-2">모듈 카탈로그</h2>
        <p className="text-surface-500 text-sm leading-relaxed">
          유료 모듈은 결제 후 영구 보유되며, 런처에서 한 번에 설치할 수 있습니다.
          무료 모듈은 별도 결제 없이 런처에서 바로 설치 가능합니다.
        </p>
      </div>

      <input
        type="search"
        value={query}
        onChange={e => setQuery(e.target.value)}
        placeholder="모듈명·설명·ID 검색…"
        className="w-full px-4 py-3 rounded-lg bg-surface-900/60 border border-white/10 text-surface-100 text-sm placeholder:text-surface-600 focus:outline-none focus:border-indigo-500/60 focus:ring-2 focus:ring-indigo-500/20 spring-transition"
      />

      {totalShown === 0 && (
        <p className="text-surface-500 text-sm text-center py-12">
          검색 결과가 없습니다.
        </p>
      )}

      {visibleCategories.map(cat => {
        const items = grouped[cat] || [];
        if (items.length === 0) return null;
        return (
          <section key={cat} className="space-y-3">
            <h3 className="text-sm font-semibold text-surface-300 uppercase tracking-wide">
              {categoryLabels[cat] || cat}
            </h3>
            <div className="space-y-3">
              {items.map(m => {
                const owned = ownedSet.has(m.id);
                const isFree = m.priceKrw === 0;
                const isFocused = m.id === focusedId;
                const imgSrc = moduleImage(m);
                return (
                  <div
                    key={m.id}
                    ref={isFocused ? focusRef : undefined}
                    className={`bezel-card spring-transition ${
                      isFocused ? "ring-2 ring-indigo-400/60 shadow-lg shadow-indigo-500/20" : ""
                    }`}
                  >
                    <div className="bezel-inner p-6">
                      <div className="flex items-start gap-4">
                        <div className="shrink-0 w-20 h-20 rounded-xl bg-white/5 border border-white/10 flex items-center justify-center overflow-hidden">
                          {imgSrc ? (
                            // eslint-disable-next-line @next/next/no-img-element
                            <img
                              src={imgSrc}
                              alt={m.name}
                              className="w-full h-full object-contain p-2"
                            />
                          ) : (
                            <span className="text-surface-600 text-[10px] uppercase tracking-wider">
                              {m.category}
                            </span>
                          )}
                        </div>
                        <div className="min-w-0 flex-1">
                          <p className="font-semibold text-surface-100">{m.name}</p>
                          <p className="text-surface-500 text-xs mt-0.5">{m.id}</p>
                          {m.description && (
                            <p className="text-surface-300 text-sm mt-3 leading-relaxed">
                              {m.description}
                            </p>
                          )}
                        </div>
                        <div className="shrink-0 text-right">
                          {owned ? (
                            <InstalledBadge />
                          ) : isFree ? (
                            <FreeBadge />
                          ) : (
                            <PayButton moduleId={m.id} priceKrw={m.priceKrw} />
                          )}
                        </div>
                      </div>
                    </div>
                  </div>
                );
              })}
            </div>
          </section>
        );
      })}
    </div>
  );

  function PayButton({ moduleId, priceKrw }: { moduleId: string; priceKrw: number }) {
    return (
      <div className="flex flex-col items-end gap-1.5">
        <span className="font-mono text-sm text-surface-100">{formatKrw(priceKrw)}</span>
        <Link
          href={`/checkout/${encodeURIComponent(moduleId)}`}
          className="px-4 py-2 rounded-full bg-gradient-to-r from-indigo-500 to-purple-500 text-white text-xs font-semibold shadow-md shadow-indigo-500/25 spring-transition cursor-pointer"
        >
          결제하기
        </Link>
      </div>
    );
  }

  function FreeBadge() {
    return (
      <div className="flex flex-col items-end gap-1.5">
        <span className="px-2.5 py-1 rounded-full bg-emerald-500/15 text-emerald-300 text-[10px] font-semibold">
          무료
        </span>
        <span className="text-xs text-surface-500 whitespace-nowrap">런처에서 설치</span>
      </div>
    );
  }

  function InstalledBadge() {
    return (
      <div className="flex flex-col items-end gap-1.5">
        <span className="px-2.5 py-1 rounded-full bg-indigo-500/20 text-indigo-200 text-[10px] font-semibold">
          ✓ 보유 중
        </span>
        <span className="text-xs text-surface-400 whitespace-nowrap">런처에서 설치</span>
      </div>
    );
  }
}
