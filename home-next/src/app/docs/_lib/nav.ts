import fs from 'node:fs/promises'
import path from 'node:path'
import matter from 'gray-matter'
import { DEFAULT_LANG, type DocLang } from './content'

const CONTENT_ROOT = path.join(process.cwd(), 'src/app/docs/_content')

export interface NavItem {
  slug: string[]
  url: string
  title: string
  order: number
  children: NavItem[]
}

interface MetaJson {
  title?: string
  order?: number
  items?: string[]
}

async function readJson<T>(p: string): Promise<T | null> {
  try {
    const raw = await fs.readFile(p, 'utf-8')
    return JSON.parse(raw) as T
  } catch {
    return null
  }
}

async function readMdxTitle(filePath: string): Promise<string | null> {
  try {
    const raw = await fs.readFile(filePath, 'utf-8')
    const { data } = matter(raw)
    return (data.title as string) ?? null
  } catch {
    return null
  }
}

async function buildNode(absDir: string, parentSlug: string[]): Promise<NavItem[]> {
  let entries: import('node:fs').Dirent[]
  try {
    entries = await fs.readdir(absDir, { withFileTypes: true })
  } catch {
    return []
  }

  const meta = (await readJson<MetaJson>(path.join(absDir, 'meta.json'))) ?? {}
  const orderHint = meta.items ?? []

  const items: NavItem[] = []

  for (const e of entries) {
    if (e.name.startsWith('.') || e.name === 'meta.json') continue
    const full = path.join(absDir, e.name)

    if (e.isDirectory()) {
      const childSlug = [...parentSlug, e.name]
      const childMeta =
        (await readJson<MetaJson>(path.join(full, 'meta.json'))) ?? {}
      const indexTitle = await readMdxTitle(path.join(full, 'index.mdx'))
      const children = await buildNode(full, childSlug)
      items.push({
        slug: childSlug,
        url: '/docs/' + childSlug.join('/'),
        title: childMeta.title ?? indexTitle ?? e.name,
        order: childMeta.order ?? 999,
        children,
      })
    } else if (e.isFile() && e.name.endsWith('.mdx') && e.name !== 'index.mdx') {
      const slug = [...parentSlug, e.name.replace(/\.mdx$/, '')]
      const title = (await readMdxTitle(full)) ?? slug[slug.length - 1]
      items.push({
        slug,
        url: '/docs/' + slug.join('/'),
        title,
        order: 999,
        children: [],
      })
    }
  }

  if (orderHint.length > 0) {
    const idx = (name: string): number => {
      const i = orderHint.indexOf(name)
      return i === -1 ? 999 : i
    }
    items.sort((a, b) => idx(a.slug[a.slug.length - 1]) - idx(b.slug[b.slug.length - 1]))
  } else {
    items.sort((a, b) => a.order - b.order || a.title.localeCompare(b.title, 'ko'))
  }

  return items
}

export async function getNavTree(lang: DocLang = DEFAULT_LANG): Promise<NavItem[]> {
  const root = path.join(CONTENT_ROOT, lang)
  return buildNode(root, [])
}
