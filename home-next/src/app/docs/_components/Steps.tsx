import type { ReactNode } from 'react'

export default function Steps({ children }: { children: ReactNode }) {
  return (
    <ol className="docs-steps relative my-6 ml-2 border-l border-white/10 pl-8 [counter-reset:step]">
      {children}
    </ol>
  )
}
