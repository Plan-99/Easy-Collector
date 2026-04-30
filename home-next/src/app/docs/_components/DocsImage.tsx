import fs from 'node:fs/promises'
import path from 'node:path'
import Image from 'next/image'

interface DocsImageProps {
  src: string
  alt: string
  caption?: string
  width?: number
  height?: number
  /**
   * Deprecated — file existence is auto-detected from `public/`.
   * The prop is still accepted for backward compatibility but ignored.
   */
  status?: 'missing'
}

async function publicFileExists(publicSrc: string): Promise<boolean> {
  // src는 항상 "/docs/.../foo.png" 형식 (Next 절대 경로)
  const rel = publicSrc.replace(/^\//, '')
  const filePath = path.join(process.cwd(), 'public', rel)
  try {
    await fs.access(filePath)
    return true
  } catch {
    return false
  }
}

function MissingFigure({
  src,
  alt,
  caption,
}: {
  src: string
  alt: string
  caption?: string
}) {
  return (
    <figure className="my-8 rounded-2xl border border-amber-500/30 bg-amber-500/5 p-6">
      <div className="flex items-center gap-3 text-sm text-amber-300">
        <span aria-hidden>📸</span>
        <span className="font-semibold">스크린샷 누락</span>
        <code className="text-xs text-amber-200/80">{src}</code>
      </div>
      <p className="mt-2 text-sm text-amber-200/70">{alt}</p>
      {caption && <p className="mt-1 text-xs text-amber-200/60">{caption}</p>}
    </figure>
  )
}

export default async function DocsImage({
  src,
  alt,
  caption,
  width = 1600,
  height = 1000,
}: DocsImageProps) {
  const exists = await publicFileExists(src)

  if (!exists) {
    return <MissingFigure src={src} alt={alt} caption={caption} />
  }

  return (
    <figure className="my-8">
      <div className="overflow-hidden rounded-2xl border border-white/10 bg-surface-900/50">
        <Image
          src={src}
          alt={alt}
          width={width}
          height={height}
          className="w-full h-auto"
        />
      </div>
      {caption && (
        <figcaption className="mt-3 text-center text-sm text-surface-500">
          {caption}
        </figcaption>
      )}
    </figure>
  )
}
