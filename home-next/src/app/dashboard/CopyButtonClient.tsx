"use client";

import { useState } from "react";

export default function CopyButtonClient({ text }: { text: string }) {
  const [copied, setCopied] = useState(false);

  const handleCopy = async () => {
    await navigator.clipboard.writeText(text);
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };

  return (
    <button
      onClick={handleCopy}
      className="px-4 py-3 rounded-xl bg-white/5 border border-white/10 text-sm font-semibold hover:bg-white/10 spring-transition"
    >
      {copied ? "복사됨!" : "복사"}
    </button>
  );
}
