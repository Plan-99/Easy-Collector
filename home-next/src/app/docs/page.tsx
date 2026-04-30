import Link from 'next/link'
import { getNavTree } from './_lib/nav'

export default async function DocsIndexPage() {
  const tree = await getNavTree()

  return (
    <article className="docs-article">
      <h1>Easy Trainer 문서</h1>
      <p className="text-lg text-surface-400">
        설치부터 추론까지, Easy Trainer의 모든 기능을 단계별로 안내합니다.
      </p>

      <div className="mt-10 grid grid-cols-1 gap-4 sm:grid-cols-2">
        {tree.map((section) => (
          <Link
            key={section.url}
            href={section.url}
            className="bezel-card group spring-transition hover:-translate-y-1"
          >
            <div className="bezel-inner p-6">
              <h3 className="font-bold text-lg text-white">{section.title}</h3>
              {section.children.length > 0 && (
                <p className="mt-2 text-sm text-surface-400">
                  {section.children
                    .slice(0, 4)
                    .map((c) => c.title)
                    .join(' · ')}
                  {section.children.length > 4 && ' …'}
                </p>
              )}
            </div>
          </Link>
        ))}
      </div>

      {tree.length === 0 && (
        <p className="mt-10 text-surface-500">
          아직 작성된 문서가 없습니다. <code>src/app/docs/_content/ko/</code> 아래에 MDX 파일을 추가해 주세요.
        </p>
      )}
    </article>
  )
}
