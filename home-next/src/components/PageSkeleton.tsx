// Generic loading skeleton used by route-level loading.tsx files. Renders
// instantly during navigation while the server component is fetching, so
// the user never sees a frozen screen.

export default function PageSkeleton({
  showHeader = true,
}: {
  showHeader?: boolean;
}) {
  return (
    <div className="min-h-dvh bg-surface-950 flex flex-col">
      <div className="flex-1 px-6 pt-16">
        <div className="max-w-3xl mx-auto py-12 animate-pulse">
          {showHeader && (
            <div className="flex items-center justify-between mb-8">
              <div className="flex items-center gap-3">
                <div className="w-10 h-10 rounded-xl bg-white/5" />
                <div className="h-5 w-24 rounded bg-white/5" />
              </div>
              <div className="h-4 w-12 rounded bg-white/5" />
            </div>
          )}
          <div className="flex gap-2 border-b border-white/5 mb-8">
            <div className="h-9 w-20 rounded bg-white/5" />
            <div className="h-9 w-16 rounded bg-white/5" />
          </div>
          <div className="space-y-6">
            <div className="bezel-card">
              <div className="bezel-inner p-8 space-y-4">
                <div className="h-5 w-32 rounded bg-white/5" />
                <div className="h-4 w-full rounded bg-white/5" />
                <div className="h-4 w-2/3 rounded bg-white/5" />
              </div>
            </div>
            <div className="bezel-card">
              <div className="bezel-inner p-8 space-y-3">
                <div className="h-5 w-40 rounded bg-white/5" />
                <div className="h-4 w-1/2 rounded bg-white/5" />
                <div className="h-16 rounded bg-white/5 mt-3" />
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
