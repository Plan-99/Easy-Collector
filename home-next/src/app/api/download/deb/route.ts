import { NextResponse } from "next/server";

export const dynamic = "force-dynamic";

export async function GET() {
  try {
    const res = await fetch(
      "https://api.github.com/repos/Plan-99/Easy-Collector-Release/releases/latest",
      {
        headers: { Accept: "application/vnd.github+json" },
        next: { revalidate: 300 },
      },
    );
    if (!res.ok) {
      return NextResponse.json({ error: "release fetch failed" }, { status: 502 });
    }
    const data = await res.json();
    const deb = data.assets?.find((a: { name: string }) => a.name.endsWith(".deb"));
    if (!deb) {
      return NextResponse.json({ error: "no .deb asset" }, { status: 404 });
    }
    return NextResponse.redirect(deb.browser_download_url, 302);
  } catch {
    return NextResponse.json({ error: "internal error" }, { status: 500 });
  }
}
