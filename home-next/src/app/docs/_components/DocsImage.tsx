import Image from 'next/image'

interface DocsImageProps {
  src: string
  alt: string
  caption?: string
  width?: number
  height?: number
  status?: 'missing'
}

export default function DocsImage({
  src,
  alt,
  caption,
  width = 1600,
  height = 1000,
  status,
}: DocsImageProps) {
  if (status === 'missing') {
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
