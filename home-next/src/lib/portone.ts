// Server-only PortOne v2 helpers. Wraps:
//   - PaymentClient (REST: getPayment, cancelPayment)
//   - Webhook signature verification
// Throws if env vars are missing so failure is loud, not silent.
import "server-only";
import { PaymentClient } from "@portone/server-sdk/payment";
import { verify as verifyWebhook } from "@portone/server-sdk/webhook";
import type { Payment as PortonePayment } from "@portone/server-sdk/payment";

function requireEnv(name: string): string {
  const v = process.env[name];
  if (!v) throw new Error(`Missing env ${name}`);
  return v;
}

export const PORTONE_STORE_ID = process.env.PORTONE_STORE_ID || "";
export const PORTONE_CHANNEL_KEY = process.env.PORTONE_CHANNEL_KEY || "";
// Optional dedicated KakaoPay channel. When set, KakaoPay payments route
// through this channel instead of the default (Galaxia). Useful when the
// default PG's test merchant doesn't have KakaoPay enabled.
export const PORTONE_CHANNEL_KEY_KAKAOPAY = process.env.PORTONE_CHANNEL_KEY_KAKAOPAY || "";

let _client: ReturnType<typeof PaymentClient> | null = null;
export function paymentClient() {
  if (_client) return _client;
  _client = PaymentClient({ secret: requireEnv("PORTONE_V2_API_SECRET") });
  return _client;
}

export type PortoneStatus = PortonePayment["status"];

export async function getPortonePayment(paymentId: string): Promise<PortonePayment> {
  return paymentClient().getPayment({ paymentId });
}

export async function cancelPortonePayment(opts: {
  paymentId: string;
  reason: string;
  amount?: number;
  requester?: "CUSTOMER" | "ADMIN";
}) {
  return paymentClient().cancelPayment({
    paymentId: opts.paymentId,
    reason: opts.reason,
    amount: opts.amount,
    requester: opts.requester,
  });
}

export async function verifyPortoneWebhook(
  payload: string,
  headers: Record<string, string | string[] | undefined>
) {
  return verifyWebhook(requireEnv("PORTONE_WEBHOOK_SECRET"), payload, headers);
}

// Build a unique paymentId for a (user, module) attempt. PortOne requires
// uniqueness — once a paymentId is PAID it can't be reused.
// Some PGs reject any non-alphanumeric character (including '_' and '-'),
// so we stick to [a-zA-Z0-9].
export function buildPaymentId(prefix = "pay") {
  const ts = Date.now().toString(36);
  const rand = Math.random().toString(36).slice(2, 10);
  return `${prefix}${ts}${rand}`;
}
