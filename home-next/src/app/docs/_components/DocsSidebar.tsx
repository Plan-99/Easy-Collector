'use client'

import Link from 'next/link'
import { usePathname } from 'next/navigation'
import type { NavItem } from '../_lib/nav'

function NavLink({ item, depth = 0 }: { item: NavItem; depth?: number }) {
  const pathname = usePathname()
  const active = pathname === item.url || pathname?.startsWith(item.url + '/')
  const hasChildren = item.children.length > 0

  return (
    <li>
      <Link
        href={item.url}
        className={[
          'block rounded-md px-3 py-1.5 text-sm spring-transition',
          depth === 0 ? 'font-semibold text-surface-200' : 'text-surface-400',
          active
            ? 'bg-indigo-500/10 text-indigo-300'
            : 'hover:bg-white/5 hover:text-white',
        ].join(' ')}
      >
        {item.title}
      </Link>
      {hasChildren && (
        <ul className="mt-1 space-y-0.5 border-l border-white/5 pl-3">
          {item.children.map((c) => (
            <NavLink key={c.url} item={c} depth={depth + 1} />
          ))}
        </ul>
      )}
    </li>
  )
}

export default function DocsSidebar({ items }: { items: NavItem[] }) {
  return (
    <nav aria-label="문서 사이드바" className="text-sm">
      <Link
        href="/docs"
        className="mb-4 block text-xs font-bold uppercase tracking-widest text-surface-500 hover:text-white"
      >
        DOCS
      </Link>
      <ul className="space-y-1">
        {items.map((it) => (
          <NavLink key={it.url} item={it} />
        ))}
      </ul>
    </nav>
  )
}
