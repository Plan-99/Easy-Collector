import type { Metadata } from 'next'
import Navbar from '@/components/Navbar'
import Footer from '@/components/Footer'
import { auth } from '@/auth'
import DocsSidebar from './_components/DocsSidebar'
import DocsTOC from './_components/DocsTOC'
import { getNavTree } from './_lib/nav'
import './docs.css'

export const metadata: Metadata = {
  title: {
    default: '문서 — Easy Trainer',
    template: '%s — Easy Trainer 문서',
  },
  description: 'Easy Trainer 사용 매뉴얼. 설치부터 추론까지 단계별 가이드.',
}

export default async function DocsLayout({
  children,
}: {
  children: React.ReactNode
}) {
  const session = await auth()
  const isLoggedIn = !!session?.user
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const isAdmin = (session?.user as any)?.role === 'admin'

  const navItems = await getNavTree()

  return (
    <>
      <Navbar isLoggedIn={isLoggedIn} isAdmin={isAdmin} />
      <div className="min-h-dvh pt-16">
        <div className="mx-auto grid max-w-7xl grid-cols-1 gap-8 px-6 md:grid-cols-[240px_1fr] md:gap-10 lg:grid-cols-[240px_1fr_220px] lg:gap-12">
          <aside className="hidden md:block">
            <div className="sticky top-20 max-h-[calc(100dvh-6rem)] overflow-y-auto py-10 pr-2">
              <DocsSidebar items={navItems} />
            </div>
          </aside>
          <main className="min-w-0 py-10">{children}</main>
          <aside className="hidden lg:block">
            <div className="sticky top-20 max-h-[calc(100dvh-6rem)] overflow-y-auto py-10 pl-2">
              <DocsTOC />
            </div>
          </aside>
        </div>
      </div>
      <Footer />
    </>
  )
}
