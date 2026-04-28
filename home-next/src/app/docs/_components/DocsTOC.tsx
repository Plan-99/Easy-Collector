'use client'

import { useEffect, useState } from 'react'

interface Heading {
  id: string
  text: string
  level: number
}

export default function DocsTOC() {
  const [headings, setHeadings] = useState<Heading[]>([])
  const [activeId, setActiveId] = useState<string | null>(null)

  useEffect(() => {
    const article = document.querySelector('article.docs-article')
    if (!article) return
    const nodes = article.querySelectorAll<HTMLElement>('h2[id], h3[id]')
    const list: Heading[] = []
    nodes.forEach((n) => {
      list.push({
        id: n.id,
        text: n.textContent?.replace(/#$/, '').trim() ?? '',
        level: n.tagName === 'H2' ? 2 : 3,
      })
    })
    // DOM measurement → state. setState here is intentional, not a sync loop.
    // eslint-disable-next-line react-hooks/set-state-in-effect
    setHeadings(list)

    if (list.length === 0) return

    const observer = new IntersectionObserver(
      (entries) => {
        const visible = entries.filter((e) => e.isIntersecting)
        if (visible.length > 0) {
          setActiveId(visible[0].target.id)
        }
      },
      { rootMargin: '-80px 0px -70% 0px', threshold: 0 },
    )
    nodes.forEach((n) => observer.observe(n))
    return () => observer.disconnect()
  }, [])

  if (headings.length === 0) return null

  return (
    <nav aria-label="목차" className="text-sm">
      <p className="mb-3 text-xs font-bold uppercase tracking-widest text-surface-500">
        목차
      </p>
      <ul className="space-y-1.5 border-l border-white/5">
        {headings.map((h) => (
          <li key={h.id} className={h.level === 3 ? 'pl-6' : 'pl-3'}>
            <a
              href={`#${h.id}`}
              className={[
                '-ml-px block border-l py-0.5 spring-transition',
                activeId === h.id
                  ? 'border-indigo-400 text-indigo-300'
                  : 'border-transparent text-surface-500 hover:text-white',
              ].join(' ')}
            >
              {h.text}
            </a>
          </li>
        ))}
      </ul>
    </nav>
  )
}
