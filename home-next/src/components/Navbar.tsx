"use client";

import { useState } from "react";
import Link from "next/link";

export default function Navbar({ isLoggedIn = false, isAdmin = false }: { isLoggedIn?: boolean; isAdmin?: boolean }) {
  const [menuOpen, setMenuOpen] = useState(false);

  return (
    <nav className="glass-nav fixed top-0 left-0 right-0 z-50">
      <div className="max-w-7xl mx-auto px-6 h-16 flex items-center justify-between">
        <Link href="/" className="flex items-center gap-2.5">
          <div className="w-8 h-8 rounded-lg bg-gradient-to-br from-indigo-500 to-purple-500 flex items-center justify-center text-xs font-extrabold tracking-tight">
            ET
          </div>
          <span className="font-[var(--font-display)] font-bold text-lg tracking-tight">
            Easy Trainer
          </span>
        </Link>

        {/* Desktop */}
        <div className="hidden md:flex items-center gap-8">
          <Link href="/#problem" className="text-sm text-surface-400 hover:text-white spring-transition">기능</Link>
          <Link href="/#services" className="text-sm text-surface-400 hover:text-white spring-transition">데모</Link>
          <Link href="/#pricing" className="text-sm text-surface-400 hover:text-white spring-transition">요금제</Link>
          <Link href="/docs" className="text-sm text-surface-400 hover:text-white spring-transition">문서</Link>
          <Link
            href={isLoggedIn ? "/dashboard" : "/auth/signin"}
            className="text-sm font-semibold px-5 py-2 rounded-full bg-white/10 hover:bg-white/15 border border-white/10 spring-transition"
          >
            {isLoggedIn ? "내 정보" : "시작하기"}
          </Link>
          {isAdmin && (
            <Link
              href="/admin"
              className="text-sm font-semibold px-5 py-2 rounded-full bg-indigo-500/20 hover:bg-indigo-500/30 border border-indigo-500/30 text-indigo-300 spring-transition"
            >
              관리자
            </Link>
          )}
        </div>

        {/* Hamburger */}
        <button
          className="md:hidden flex flex-col gap-1.5 p-1"
          aria-label="메뉴"
          onClick={() => setMenuOpen(!menuOpen)}
        >
          <span
            className="block w-5 h-0.5 bg-white rounded spring-transition origin-center"
            style={menuOpen ? { transform: "rotate(45deg) translate(3px, 3px)" } : {}}
          />
          <span
            className="block w-5 h-0.5 bg-white rounded spring-transition"
            style={menuOpen ? { opacity: 0 } : {}}
          />
          <span
            className="block w-5 h-0.5 bg-white rounded spring-transition origin-center"
            style={menuOpen ? { transform: "rotate(-45deg) translate(3px, -3px)" } : {}}
          />
        </button>
      </div>

      {/* Mobile menu */}
      <div
        className={`md:hidden overflow-hidden spring-transition ${menuOpen ? "mobile-open" : ""}`}
        style={{ maxHeight: menuOpen ? 300 : 0 }}
      >
        <div className="px-6 pb-6 pt-2 flex flex-col gap-4 border-t border-white/5">
          <Link href="/#problem" className="mobile-link text-surface-300 hover:text-white py-1" onClick={() => setMenuOpen(false)}>기능</Link>
          <Link href="/#services" className="mobile-link text-surface-300 hover:text-white py-1" onClick={() => setMenuOpen(false)}>데모</Link>
          <Link href="/#pricing" className="mobile-link text-surface-300 hover:text-white py-1" onClick={() => setMenuOpen(false)}>요금제</Link>
          <Link href="/docs" className="mobile-link text-surface-300 hover:text-white py-1" onClick={() => setMenuOpen(false)}>문서</Link>
          <Link href={isLoggedIn ? "/dashboard" : "/auth/signin"} className="mobile-link text-sm font-semibold px-5 py-2.5 rounded-full bg-white/10 border border-white/10 text-center mt-1">
            {isLoggedIn ? "내 정보" : "시작하기"}
          </Link>
          {isAdmin && (
            <Link href="/admin" className="mobile-link text-sm font-semibold px-5 py-2.5 rounded-full bg-indigo-500/20 border border-indigo-500/30 text-indigo-300 text-center" onClick={() => setMenuOpen(false)}>
              관리자
            </Link>
          )}
        </div>
      </div>
    </nav>
  );
}
