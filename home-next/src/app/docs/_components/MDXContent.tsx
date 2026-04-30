import { MDXRemote } from 'next-mdx-remote/rsc'
import rehypeSlug from 'rehype-slug'
import rehypeAutolinkHeadings from 'rehype-autolink-headings'
import remarkGfm from 'remark-gfm'
import DocsImage from './DocsImage'
import Callout from './Callout'
import Steps from './Steps'

const mdxComponents = {
  DocsImage,
  Callout,
  Steps,
}

export default function MDXContent({ source }: { source: string }) {
  return (
    <MDXRemote
      source={source}
      components={mdxComponents}
      options={{
        parseFrontmatter: false,
        mdxOptions: {
          remarkPlugins: [remarkGfm],
          rehypePlugins: [
            rehypeSlug,
            [
              rehypeAutolinkHeadings,
              {
                behavior: 'append',
                properties: { className: 'docs-anchor', ariaHidden: 'true', tabIndex: -1 },
                content: { type: 'text', value: ' #' },
              },
            ],
          ],
        },
      }}
    />
  )
}
