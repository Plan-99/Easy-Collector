"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";

const TABS = [
  { href: "/dashboard", label: "내 정보" },
  { href: "/dashboard/modules", label: "모듈" },
];

export default function DashboardTabs() {
  const pathname = usePathname() || "";
  return (
    <nav className="flex gap-1 border-b border-white/10 mb-8">
      {TABS.map(t => {
        const active = pathname === t.href;
        return (
          <Link
            key={t.href}
            href={t.href}
            className={`px-4 py-2.5 text-sm font-semibold border-b-2 -mb-px spring-transition ${
              active
                ? "border-indigo-400 text-white"
                : "border-transparent text-surface-400 hover:text-surface-100"
            }`}
          >
            {t.label}
          </Link>
        );
      })}
    </nav>
  );
}
