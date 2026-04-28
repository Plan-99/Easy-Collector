"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";

const TABS: { href: string; label: string; matchPrefix: string }[] = [
  { href: "/admin/users", label: "사용자 관리", matchPrefix: "/admin/users" },
  { href: "/admin/modules", label: "모듈 관리", matchPrefix: "/admin/modules" },
];

export default function AdminTabs() {
  const pathname = usePathname() || "";
  return (
    <nav className="flex gap-1 border-b border-white/10 mb-6">
      {TABS.map(t => {
        const active = pathname === t.href || pathname.startsWith(t.matchPrefix + "/");
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
