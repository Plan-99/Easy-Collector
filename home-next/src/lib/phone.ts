// Auto-format Korean phone numbers as the user types.
//   "01082222222"   → "010-8222-2222"
//   "0212345678"    → "02-1234-5678"
//   "0312345678"    → "031-234-5678" / "031-1234-5678"
//
// Rules:
// - 02 (Seoul):     2-3-4 or 2-4-4
// - 010/011/...:    3-3-4 or 3-4-4

export function formatPhoneKr(input: string): string {
  const d = input.replace(/\D/g, "").slice(0, 11);
  if (d.length === 0) return "";

  if (d.startsWith("02")) {
    if (d.length <= 2) return d;
    if (d.length <= 5) return `${d.slice(0, 2)}-${d.slice(2)}`;
    if (d.length <= 9) return `${d.slice(0, 2)}-${d.slice(2, 5)}-${d.slice(5)}`;
    return `${d.slice(0, 2)}-${d.slice(2, 6)}-${d.slice(6, 10)}`;
  }

  if (d.length <= 3) return d;
  if (d.length <= 6) return `${d.slice(0, 3)}-${d.slice(3)}`;
  if (d.length <= 10) return `${d.slice(0, 3)}-${d.slice(3, 6)}-${d.slice(6)}`;
  return `${d.slice(0, 3)}-${d.slice(3, 7)}-${d.slice(7)}`;
}
