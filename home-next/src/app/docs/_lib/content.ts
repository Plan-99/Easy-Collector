import fs from 'node:fs/promises'
import path from 'node:path'
import matter from 'gray-matter'

export const DEFAULT_LANG = 'ko'
export const SUPPORTED_LANGS = ['ko'] as const
export type DocLang = (typeof SUPPORTED_LANGS)[number]

const CONTENT_ROOT = path.join(process.cwd(), 'src/app/docs/_content')

export interface DocFrontmatter {
  title: string
  description?: string
  section?: string
  order?: number
  related?: string[]
  sourceRefs?: string[]
  lastVerifiedVersion?: string
  lang?: DocLang
}

export interface DocPage {
  slug: string[]
  url: string
  frontmatter: DocFrontmatter
  content: string
  filePath: string
}

async function fileExists(p: string): Promise<boolean> {
  try {
    await fs.access(p)
    return true
  } catch {
    return false
  }
}

function langRoot(lang: DocLang): string {
  return path.join(CONTENT_ROOT, lang)
}

export async function getDocPage(
  slug: string[],
  lang: DocLang = DEFAULT_LANG,
): Promise<DocPage | null> {
  const safeSlug = slug.filter((s) => s && !s.startsWith('.') && !s.includes('/'))
  if (safeSlug.length === 0) return null

  const candidates = [
    path.join(langRoot(lang), ...safeSlug, 'index.mdx'),
    path.join(langRoot(lang), ...safeSlug.slice(0, -1), `${safeSlug[safeSlug.length - 1]}.mdx`),
  ]

  for (const filePath of candidates) {
    if (!(await fileExists(filePath))) continue
    const raw = await fs.readFile(filePath, 'utf-8')
    const parsed = matter(raw)
    return {
      slug: safeSlug,
      url: '/docs/' + safeSlug.join('/'),
      frontmatter: parsed.data as DocFrontmatter,
      content: parsed.content,
      filePath,
    }
  }
  return null
}

async function walk(dir: string): Promise<string[]> {
  const out: string[] = []
  let entries: import('node:fs').Dirent[]
  try {
    entries = await fs.readdir(dir, { withFileTypes: true })
  } catch {
    return out
  }
  for (const e of entries) {
    const full = path.join(dir, e.name)
    if (e.isDirectory()) {
      out.push(...(await walk(full)))
    } else if (e.isFile() && e.name.endsWith('.mdx')) {
      out.push(full)
    }
  }
  return out
}

export async function listAllSlugs(lang: DocLang = DEFAULT_LANG): Promise<string[][]> {
  const root = langRoot(lang)
  const files = await walk(root)
  return files.map((f) => {
    const rel = path.relative(root, f).replace(/\\/g, '/')
    const noExt = rel.replace(/\.mdx$/, '')
    const parts = noExt.split('/')
    if (parts[parts.length - 1] === 'index') parts.pop()
    return parts
  })
}
