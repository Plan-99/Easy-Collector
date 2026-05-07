"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import * as PortOne from "@portone/browser-sdk/v2";
import { openPostcode } from "@/lib/postcode";
import { formatPhoneKr } from "@/lib/phone";

type Variant = {
  id: string;
  name: string;
  priceKrw: number;
  stock: number;
};

type SavedAddress = {
  id: string;
  label: string | null;
  recipientName: string;
  phone: string;
  postalCode: string;
  addressLine1: string;
  addressLine2: string | null;
  requestNote: string | null;
  isDefault: boolean;
};

type Props = {
  productId: string;
  productName: string;
  priceKrw: number;
  maxQuantity: number; // ignored if variants is non-empty (uses variant.stock)
  variants: Variant[];
  savedAddresses: SavedAddress[];
  buyerEmail: string;
  buyerName: string;
  customerId: string;
  storeId: string;
  channelKey: string;
};

type Phase = "idle" | "initiating" | "paying" | "verifying" | "done" | "error";

function formatKrw(amountKrw: number) {
  return new Intl.NumberFormat("ko-KR", {
    style: "currency",
    currency: "KRW",
    maximumFractionDigits: 0,
  }).format(amountKrw);
}

export default function OrderForm(props: Props) {
  const router = useRouter();
  const hasVariants = props.variants.length > 0;
  const [variantId, setVariantId] = useState<string>(
    hasVariants && props.variants[0].stock > 0 ? props.variants[0].id : ""
  );
  const selected = hasVariants ? props.variants.find(v => v.id === variantId) : null;
  const unitPrice = selected ? selected.priceKrw : props.priceKrw;
  const maxQty = selected ? selected.stock : props.maxQuantity;
  const [quantity, setQuantity] = useState(1);
  // Saved address picker — defaults to default address if available.
  const defaultSaved = props.savedAddresses.find(a => a.isDefault) || props.savedAddresses[0];
  const [selectedSavedId, setSelectedSavedId] = useState<string>(
    defaultSaved ? defaultSaved.id : "new"
  );
  const [recipientName, setRecipientName] = useState(defaultSaved?.recipientName || "");
  const [phone, setPhone] = useState(formatPhoneKr(defaultSaved?.phone || ""));
  const [postalCode, setPostalCode] = useState(defaultSaved?.postalCode || "");
  const [line1, setLine1] = useState(defaultSaved?.addressLine1 || "");
  const [line2, setLine2] = useState(defaultSaved?.addressLine2 || "");
  const [requestNote, setRequestNote] = useState(defaultSaved?.requestNote || "");
  const [saveAddress, setSaveAddress] = useState(false);
  const [agreed, setAgreed] = useState(false);

  function applySavedAddress(id: string) {
    setSelectedSavedId(id);
    if (id === "new") {
      setRecipientName("");
      setPhone("");
      setPostalCode("");
      setLine1("");
      setLine2("");
      setRequestNote("");
      return;
    }
    const a = props.savedAddresses.find(x => x.id === id);
    if (!a) return;
    setRecipientName(a.recipientName);
    setPhone(formatPhoneKr(a.phone));
    setPostalCode(a.postalCode);
    setLine1(a.addressLine1);
    setLine2(a.addressLine2 || "");
    setRequestNote(a.requestNote || "");
    setSaveAddress(false);
  }
  const [phase, setPhase] = useState<Phase>("idle");
  const [error, setError] = useState<string | null>(null);
  const [orderId, setOrderId] = useState<string | null>(null);

  const total = unitPrice * quantity;
  const formValid =
    (!hasVariants || (!!variantId && (selected?.stock ?? 0) > 0)) &&
    !!recipientName.trim() &&
    !!phone.trim() &&
    !!postalCode.trim() &&
    !!line1.trim() &&
    agreed &&
    quantity > 0 &&
    quantity <= maxQty;
  const busy = phase === "initiating" || phase === "paying" || phase === "verifying";

  async function handlePay() {
    setError(null);
    setPhase("initiating");
    try {
      // Best-effort save: not fatal if it fails
      if (saveAddress && selectedSavedId === "new") {
        try {
          await fetch("/api/account/addresses", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({
              recipientName: recipientName.trim(),
              phone: phone.trim(),
              postalCode: postalCode.trim(),
              addressLine1: line1.trim(),
              addressLine2: line2.trim() || null,
              requestNote: requestNote.trim() || null,
            }),
          });
        } catch {
          // ignore — checkout proceeds
        }
      }
      const initRes = await fetch("/api/orders/checkout", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          productId: props.productId,
          variantId: hasVariants ? variantId : undefined,
          quantity,
          address: {
            recipientName: recipientName.trim(),
            phone: phone.trim(),
            postalCode: postalCode.trim(),
            line1: line1.trim(),
            line2: line2.trim() || undefined,
            requestNote: requestNote.trim() || undefined,
          },
        }),
      });
      if (!initRes.ok) {
        const b = await initRes.json().catch(() => ({}));
        throw new Error(b?.error || "주문 준비 실패");
      }
      const { paymentId, amountKrw, orderName, orderId: oid } = (await initRes.json()) as {
        paymentId: string;
        amountKrw: number;
        orderName: string;
        orderId: string;
      };
      setOrderId(oid);

      setPhase("paying");
      const paymentRequest = {
        storeId: props.storeId,
        channelKey: props.channelKey,
        paymentId,
        orderName,
        totalAmount: amountKrw,
        currency: "CURRENCY_KRW",
        payMethod: "CARD",
        customer: {
          customerId: props.customerId,
          email: props.buyerEmail || undefined,
          fullName: props.buyerName || undefined,
        },
        customData: { orderId: oid, productId: props.productId },
        windowType: { mobile: "REDIRECTION" },
        redirectUrl:
          typeof window !== "undefined"
            ? `${window.location.origin}/dashboard/orders/${oid}?paymentId=${encodeURIComponent(paymentId)}`
            : undefined,
      };
      const result = await PortOne.requestPayment(
        paymentRequest as unknown as Parameters<typeof PortOne.requestPayment>[0]
      );
      if (!result || result.code != null) {
        throw new Error(result?.message || "결제가 취소되었거나 실패했습니다.");
      }

      setPhase("verifying");
      const completeRes = await fetch("/api/orders/complete", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ paymentId }),
      });
      if (!completeRes.ok) {
        const b = await completeRes.json().catch(() => ({}));
        throw new Error(b?.error || "결제 검증 실패");
      }
      setPhase("done");
    } catch (err) {
      setPhase("error");
      setError(err instanceof Error ? err.message : "알 수 없는 오류");
    }
  }

  if (phase === "done" && orderId) {
    return (
      <div className="bezel-card mt-4">
        <div className="bezel-inner p-8 space-y-3 text-center">
          <p className="font-bold text-emerald-300 text-lg">결제가 완료되었습니다 ✓</p>
          <p className="text-surface-300 text-sm">
            주문이 접수되었습니다. 발송이 시작되면 등록하신 연락처로 안내드립니다.
          </p>
          <div className="flex gap-2 justify-center mt-4">
            <button
              onClick={() => router.push(`/dashboard/orders/${orderId}`)}
              className="px-5 py-2.5 rounded-full bg-gradient-to-r from-indigo-500 to-purple-500 text-white text-sm font-semibold cursor-pointer"
            >
              주문 상세 보기
            </button>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="bezel-card mt-4">
      <div className="bezel-inner p-8 space-y-5">
        <h3 className="font-bold text-lg">주문하기</h3>

        {hasVariants && (
          <div className="space-y-2">
            <p className="text-xs font-semibold text-surface-300">옵션 선택 *</p>
            <div className="grid grid-cols-1 sm:grid-cols-2 gap-2">
              {props.variants.map(v => {
                const isSelected = v.id === variantId;
                const oos = v.stock <= 0;
                return (
                  <label
                    key={v.id}
                    className={`relative cursor-pointer rounded-xl border p-3 spring-transition ${
                      oos
                        ? "opacity-50 cursor-not-allowed bg-surface-900/40"
                        : isSelected
                          ? "bg-gradient-to-b from-indigo-500/30 to-purple-500/15 border-indigo-400/60"
                          : "border-white/10 bg-surface-900/50 hover:border-white/20"
                    }`}
                  >
                    <input
                      type="radio"
                      name="variant"
                      value={v.id}
                      disabled={oos}
                      checked={isSelected}
                      onChange={() => {
                        setVariantId(v.id);
                        setQuantity(1);
                      }}
                      className="sr-only"
                    />
                    <div className="flex items-center justify-between">
                      <span className="font-semibold text-sm text-surface-100">{v.name}</span>
                      <span className="font-mono text-xs">{formatKrw(v.priceKrw)}</span>
                    </div>
                    <p className="text-[10px] text-surface-400 mt-1">
                      {oos ? "품절" : `재고 ${v.stock}개`}
                    </p>
                  </label>
                );
              })}
            </div>
          </div>
        )}

        <div className="grid grid-cols-2 gap-4">
          <Field label="수량" required>
            <div className="flex items-center gap-2">
              <button
                type="button"
                onClick={() => setQuantity(q => Math.max(1, q - 1))}
                className="w-10 h-10 rounded-lg bg-white/5 border border-white/10 hover:bg-white/10"
              >
                −
              </button>
              <input
                type="number"
                value={quantity}
                onChange={e => {
                  const n = Math.max(1, Math.min(maxQty, Number(e.target.value) || 1));
                  setQuantity(n);
                }}
                min={1}
                max={maxQty}
                disabled={maxQty <= 0}
                className="flex-1 px-3 py-2 rounded-lg bg-surface-900/60 border border-white/10 text-surface-100 text-sm text-center font-mono disabled:opacity-50"
              />
              <button
                type="button"
                onClick={() => setQuantity(q => Math.min(maxQty, q + 1))}
                className="w-10 h-10 rounded-lg bg-white/5 border border-white/10 hover:bg-white/10"
              >
                +
              </button>
            </div>
          </Field>
          <div className="flex flex-col items-end justify-end">
            <span className="text-xs text-surface-500">총 결제 금액</span>
            <span className="font-mono font-bold text-2xl">{formatKrw(total)}</span>
          </div>
        </div>

        <div className="border-t border-white/5 pt-4 space-y-4">
          <p className="text-sm font-semibold text-surface-200">배송지</p>

          {props.savedAddresses.length > 0 && (
            <Field label="저장된 배송지">
              <select
                value={selectedSavedId}
                onChange={e => applySavedAddress(e.target.value)}
                className={inputCls}
              >
                {props.savedAddresses.map(a => (
                  <option key={a.id} value={a.id}>
                    {a.isDefault ? "★ " : ""}
                    {a.label ? `[${a.label}] ` : ""}
                    {a.recipientName} · {a.phone} · {a.addressLine1}
                  </option>
                ))}
                <option value="new">+ 새 배송지 입력</option>
              </select>
            </Field>
          )}

          <div className="grid grid-cols-2 gap-3">
            <Field label="받는 사람" required>
              <input
                type="text"
                value={recipientName}
                onChange={e => setRecipientName(e.target.value)}
                className={inputCls}
              />
            </Field>
            <Field label="연락처" required>
              <input
                type="tel"
                value={phone}
                onChange={e => setPhone(formatPhoneKr(e.target.value))}
                placeholder="010-0000-0000"
                inputMode="numeric"
                maxLength={13}
                className={inputCls}
              />
            </Field>
          </div>
          <Field label="우편번호" required>
            <div className="flex gap-2">
              <input
                type="text"
                value={postalCode}
                onChange={e => setPostalCode(e.target.value)}
                placeholder="00000"
                className={inputCls + " font-mono"}
                maxLength={6}
                readOnly
              />
              <button
                type="button"
                onClick={async () => {
                  const r = await openPostcode();
                  if (!r) return;
                  setPostalCode(r.zonecode);
                  setLine1(r.roadAddress);
                }}
                className="shrink-0 px-4 py-2 rounded-lg bg-indigo-500/20 hover:bg-indigo-500/30 border border-indigo-500/30 text-indigo-200 text-xs font-semibold whitespace-nowrap"
              >
                주소 검색
              </button>
            </div>
          </Field>
          <Field label="주소" required>
            <input
              type="text"
              value={line1}
              onChange={e => setLine1(e.target.value)}
              placeholder="주소 검색을 눌러주세요"
              className={inputCls}
              readOnly
            />
          </Field>
          <Field label="상세 주소">
            <input
              type="text"
              value={line2}
              onChange={e => setLine2(e.target.value)}
              placeholder="동·호수·기타"
              className={inputCls}
            />
          </Field>
          <Field label="배송 요청 사항">
            <input
              type="text"
              value={requestNote}
              onChange={e => setRequestNote(e.target.value)}
              placeholder="문 앞에 두세요 등"
              className={inputCls}
            />
          </Field>
        </div>

        {selectedSavedId === "new" && (
          <label className="flex items-center gap-2 text-sm text-surface-300 cursor-pointer">
            <input
              type="checkbox"
              className="w-4 h-4 accent-indigo-500"
              checked={saveAddress}
              onChange={e => setSaveAddress(e.target.checked)}
            />
            <span>이 배송지를 저장하기 (다음 주문에서 자동 입력)</span>
          </label>
        )}

        <label className="flex items-center gap-2 text-sm text-surface-300 cursor-pointer pt-3 border-t border-white/5">
          <input
            type="checkbox"
            className="w-4 h-4 accent-indigo-500"
            checked={agreed}
            onChange={e => setAgreed(e.target.checked)}
          />
          <span>구매 조건 및 환불 정책에 동의합니다.</span>
        </label>

        {error && (
          <p className="text-sm text-red-400 bg-red-500/10 border border-red-500/20 rounded-lg px-3 py-2">
            {error}
          </p>
        )}

        <button
          type="button"
          onClick={handlePay}
          disabled={!formValid || busy}
          className="w-full py-3.5 rounded-full bg-gradient-to-r from-indigo-500 to-purple-500 text-white font-semibold text-sm shadow-lg shadow-indigo-500/25 disabled:opacity-40 disabled:cursor-not-allowed cursor-pointer"
        >
          {phase === "idle" && `${formatKrw(total)} 결제하기`}
          {phase === "initiating" && "주문 준비 중…"}
          {phase === "paying" && "결제 진행 중…"}
          {phase === "verifying" && "결제 확인 중…"}
          {phase === "error" && "다시 시도"}
        </button>
        <p className="text-xs text-surface-500 text-center">
          결제창에서 카드·카카오페이·네이버페이 등 원하는 수단을 선택할 수 있습니다.
        </p>
      </div>
    </div>
  );
}

const inputCls =
  "w-full px-3 py-2 rounded-lg bg-surface-900/60 border border-white/10 text-surface-100 text-sm placeholder:text-surface-600 focus:outline-none focus:border-indigo-500/60 focus:ring-2 focus:ring-indigo-500/20";

function Field({
  label,
  required,
  children,
}: {
  label: string;
  required?: boolean;
  children: React.ReactNode;
}) {
  return (
    <label className="block">
      <span className="text-xs font-semibold text-surface-300">
        {label}
        {required && <span className="text-red-400 ml-0.5">*</span>}
      </span>
      <div className="mt-1.5">{children}</div>
    </label>
  );
}
