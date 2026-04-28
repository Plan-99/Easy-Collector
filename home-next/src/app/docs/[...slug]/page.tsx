import { notFound } from 'next/navigation'
import Link from 'next/link'
import { getDocPage, listAllSlugs } from '../_lib/content'
import MDXContent from '../_components/MDXContent'

interface DocRouteParams {
  slug: string[]
}

export async function generateStaticParams(): Promise<DocRouteParams[]> {
  const slugs = await listAllSlugs()
  return slugs
    .filter((s) => s.length > 0)
    .map((slug) => ({ slug }))
}

export async function generateMetadata({
  params,
}: {
  params: Promise<DocRouteParams>
}) {
  const { slug } = await params
  const doc = await getDocPage(slug)
  if (!doc) return {}
  return {
    title: doc.frontmatter.title,
    description: doc.frontmatter.description,
  }
}

export default async function DocSlugPage({
  params,
}: {
  params: Promise<DocRouteParams>
}) {
  const { slug } = await params
  const doc = await getDocPage(slug)
  if (!doc) notFound()

  return (
    <article className="docs-article">
      <header className="mb-10">
        <p className="text-xs font-bold uppercase tracking-widest text-indigo-400">
          {slug[0]}
        </p>
        <h1>{doc.frontmatter.title}</h1>
        {doc.frontmatter.description && (
          <p className="mt-3 text-lg text-surface-400">
            {doc.frontmatter.description}
          </p>
        )}
        {doc.frontmatter.lastVerifiedVersion && (
          <p className="mt-2 text-xs text-surface-600">
            최근 확인 버전: v{doc.frontmatter.lastVerifiedVersion}
          </p>
        )}
      </header>

      <MDXContent source={doc.content} />

      {doc.frontmatter.related && doc.frontmatter.related.length > 0 && (
        <section className="mt-16 border-t border-white/5 pt-8">
          <h2 className="!mt-0 !border-0 !pt-0">관련 문서</h2>
          <ul>
            {doc.frontmatter.related.map((href) => (
              <li key={href}>
                <Link href={href}>{href}</Link>
              </li>
            ))}
          </ul>
        </section>
      )}
    </article>
  )
}
