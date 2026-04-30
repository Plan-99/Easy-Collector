import type { ReactNode } from 'react'

type CalloutType = 'tip' | 'warning' | 'info' | 'danger'

interface CalloutProps {
  type?: CalloutType
  title?: string
  children: ReactNode
}

const STYLES: Record<CalloutType, { border: string; bg: string; text: string; icon: string }> = {
  tip: {
    border: 'border-emerald-500/30',
    bg: 'bg-emerald-500/5',
    text: 'text-emerald-300',
    icon: '💡',
  },
  warning: {
    border: 'border-amber-500/30',
    bg: 'bg-amber-500/5',
    text: 'text-amber-300',
    icon: '⚠️',
  },
  info: {
    border: 'border-indigo-500/30',
    bg: 'bg-indigo-500/5',
    text: 'text-indigo-300',
    icon: 'ℹ️',
  },
  danger: {
    border: 'border-rose-500/30',
    bg: 'bg-rose-500/5',
    text: 'text-rose-300',
    icon: '🛑',
  },
}

export default function Callout({ type = 'tip', title, children }: CalloutProps) {
  const s = STYLES[type]
  return (
    <aside className={`my-6 rounded-2xl border ${s.border} ${s.bg} px-5 py-4`}>
      <div className={`flex items-center gap-2 ${s.text} text-sm font-semibold`}>
        <span aria-hidden>{s.icon}</span>
        {title ?? type.toUpperCase()}
      </div>
      <div className="mt-2 text-surface-200 text-[0.95rem] leading-relaxed [&_p]:my-1 [&_code]:text-surface-100">
        {children}
      </div>
    </aside>
  )
}
