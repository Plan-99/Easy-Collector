"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";

type Variant = {
  name: string;
  sku: string;
  priceKrw: number;
  stock: number;
  active: boolean;
};

type Initial = {
  id?: string;
  sku: string;
  name: string;
  description: string;
  priceKrw: number;
  stock: number;
  weightG: number | null;
  leadTimeDays: number | null;
  imageUrl: string;
  active: boolean;
  variants: Variant[];
};

export default function ProductForm({ initial }: { initial: Initial }) {
  const router = useRouter();
  const isEdit = !!initial.id;
  const [sku, setSku] = useState(initial.sku);
  const [name, setName] = useState(initial.name);
  const [description, setDescription] = useState(initial.description);
  const [priceKrw, setPriceKrw] = useState(initial.priceKrw);
  const [stock, setStock] = useState(initial.stock);
  const [weightG, setWeightG] = useState<number | "">(initial.weightG ?? "");
  const [leadTimeDays, setLeadTimeDays] = useState<number | "">(initial.leadTimeDays ?? "");
  const [imageUrl, setImageUrl] = useState(initial.imageUrl);
  const [uploadingImage, setUploadingImage] = useState(false);
  const [uploadError, setUploadError] = useState<string | null>(null);
  const [active, setActive] = useState(initial.active);
  const [variants, setVariants] = useState<Variant[]>(initial.variants);
  const [hasVariants, setHasVariants] = useState(initial.variants.length > 0);
  const [saving, setSaving] = useState(false);
  const [deleting, setDeleting] = useState(false);
  const [error, setError] = useState<string | null>(null);

  async function uploadImage(file: File) {
    setUploadingImage(true);
    setUploadError(null);
    try {
      const res = await fetch(
        `/api/admin/upload?filename=${encodeURIComponent(file.name)}`,
        {
          method: "POST",
          headers: { "Content-Type": file.type || "application/octet-stream" },
          body: file,
        }
      );
      if (!res.ok) {
        const b = await res.json().catch(() => ({}));
        throw new Error(
          b?.error === "BLOB_NOT_CONFIGURED"
            ? "Vercel Blob 저장소가 설정되지 않았습니다. 관리자에게 문의하세요."
            : b?.error === "FILE_TOO_LARGE"
              ? "이미지는 5MB 이하만 업로드할 수 있습니다."
              : "업로드 실패"
        );
      }
      const { url } = (await res.json()) as { url: string };
      setImageUrl(url);
    } catch (err) {
      setUploadError(err instanceof Error ? err.message : "알 수 없는 오류");
    } finally {
      setUploadingImage(false);
    }
  }

  async function save() {
    setSaving(true);
    setError(null);
    try {
      const body = {
        sku: hasVariants ? null : sku.trim(),
        name: name.trim(),
        description: description.trim() || null,
        priceKrw: Math.floor(priceKrw),
        stock: Math.floor(stock),
        weightG: weightG === "" ? null : Math.floor(weightG),
        leadTimeDays: leadTimeDays === "" ? null : Math.floor(leadTimeDays),
        imageUrl: imageUrl.trim() || null,
        active,
        variants: hasVariants
          ? variants
              .filter(v => v.name.trim() && v.sku.trim())
              .map(v => ({
                name: v.name.trim(),
                sku: v.sku.trim(),
                priceKrw: Math.floor(v.priceKrw),
                stock: Math.floor(v.stock),
                active: v.active,
              }))
          : [],
      };
      const res = await fetch(
        isEdit ? `/api/admin/products/${initial.id}` : "/api/admin/products",
        {
          method: isEdit ? "PATCH" : "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(body),
        }
      );
      if (!res.ok) {
        const b = await res.json().catch(() => ({}));
        throw new Error(b?.error || "저장 실패");
      }
      router.push("/admin/products");
      router.refresh();
    } catch (err) {
      setError(err instanceof Error ? err.message : "알 수 없는 오류");
    } finally {
      setSaving(false);
    }
  }

  async function remove() {
    if (!isEdit) return;
    if (!confirm(`"${name}" 상품을 삭제하시겠습니까? 주문이 한 건이라도 있으면 비활성화만 됩니다.`)) return;
    setDeleting(true);
    try {
      const res = await fetch(`/api/admin/products/${initial.id}`, { method: "DELETE" });
      if (!res.ok) throw new Error("삭제 실패");
      router.push("/admin/products");
      router.refresh();
    } catch (err) {
      alert(err instanceof Error ? err.message : "알 수 없는 오류");
      setDeleting(false);
    }
  }

  return (
    <div className="bezel-card">
      <div className="bezel-inner p-8 space-y-5">
        <Field label="상품명" required>
          <input
            type="text"
            value={name}
            onChange={e => setName(e.target.value)}
            className={inputCls}
          />
        </Field>
        <div className="grid grid-cols-2 gap-4">
          <Field
            label="SKU"
            required={!hasVariants}
            hint={hasVariants ? "옵션 사용 시 비워두면 옵션 SKU만 사용됩니다" : "고유 코드 (예: leader-piper-v1)"}
          >
            <input
              type="text"
              value={sku}
              onChange={e => setSku(e.target.value)}
              disabled={hasVariants}
              placeholder={hasVariants ? "옵션 SKU만 사용" : ""}
              className={inputCls + (hasVariants ? " opacity-50 cursor-not-allowed" : "")}
            />
          </Field>
          <Field
            label="기본 가격 (KRW)"
            required={!hasVariants}
            hint={hasVariants ? "옵션 가격이 우선 적용됩니다" : ""}
          >
            <input
              type="number"
              min={0}
              step={1000}
              value={priceKrw}
              onChange={e => setPriceKrw(Number(e.target.value) || 0)}
              className={inputCls + " font-mono text-right"}
            />
          </Field>
        </div>
        <Field label="설명">
          <textarea
            value={description}
            onChange={e => setDescription(e.target.value)}
            rows={4}
            className={inputCls + " resize-none"}
          />
        </Field>
        <div className="grid grid-cols-3 gap-4">
          <Field label="재고" hint={hasVariants ? "옵션별 재고 사용" : ""}>
            <input
              type="number"
              min={0}
              value={stock}
              onChange={e => setStock(Number(e.target.value) || 0)}
              disabled={hasVariants}
              className={inputCls + (hasVariants ? " opacity-50 cursor-not-allowed" : "")}
            />
          </Field>
          <Field label="무게 (g)" hint="배송비 계산용">
            <input
              type="number"
              min={0}
              value={weightG}
              onChange={e => setWeightG(e.target.value === "" ? "" : Number(e.target.value))}
              className={inputCls}
            />
          </Field>
          <Field label="발송 소요일" hint="결제 후 며칠">
            <input
              type="number"
              min={0}
              value={leadTimeDays}
              onChange={e =>
                setLeadTimeDays(e.target.value === "" ? "" : Number(e.target.value))
              }
              className={inputCls}
            />
          </Field>
        </div>
        <Field label="이미지" hint="파일 업로드 또는 URL 직접 입력 (최대 5MB)">
          <div className="space-y-2">
            <div className="flex items-center gap-2">
              <label
                className={`px-4 py-2 rounded-lg bg-indigo-500/20 hover:bg-indigo-500/30 border border-indigo-500/30 text-indigo-200 text-xs font-semibold cursor-pointer ${uploadingImage ? "opacity-50 cursor-wait" : ""}`}
              >
                {uploadingImage ? "업로드 중…" : "파일 선택"}
                <input
                  type="file"
                  accept="image/*"
                  className="sr-only"
                  disabled={uploadingImage}
                  onChange={e => {
                    const f = e.target.files?.[0];
                    if (f) uploadImage(f);
                    e.target.value = "";
                  }}
                />
              </label>
              {imageUrl && (
                <button
                  type="button"
                  onClick={() => setImageUrl("")}
                  className="text-xs text-red-300 hover:text-red-200 underline"
                >
                  이미지 제거
                </button>
              )}
            </div>
            <input
              type="text"
              value={imageUrl}
              onChange={e => setImageUrl(e.target.value)}
              placeholder="또는 직접 URL 붙여넣기 (https://…)"
              className={inputCls}
            />
            {uploadError && (
              <p className="text-xs text-red-400 bg-red-500/10 border border-red-500/20 rounded-md px-3 py-2">
                {uploadError}
              </p>
            )}
          </div>
        </Field>
        {imageUrl && (
          // eslint-disable-next-line @next/next/no-img-element
          <img
            src={imageUrl}
            alt="preview"
            className="h-32 object-contain bg-white/5 rounded-lg p-2 border border-white/10"
          />
        )}
        {/* Variants */}
        <div className="border-t border-white/5 pt-5 space-y-3">
          <label className="flex items-center gap-2 text-sm cursor-pointer select-none">
            <input
              type="checkbox"
              className="w-4 h-4 accent-indigo-500"
              checked={hasVariants}
              onChange={e => {
                const next = e.target.checked;
                setHasVariants(next);
                if (next && variants.length === 0) {
                  setVariants([
                    { name: "", sku: "", priceKrw, stock: 0, active: true },
                  ]);
                }
              }}
            />
            <span className="text-surface-200 font-semibold text-sm">
              옵션 사용 (예: 색상·사이즈·버전)
            </span>
          </label>
          {hasVariants && (
            <div className="space-y-2">
              <div className="grid grid-cols-[1fr_1fr_120px_80px_60px_30px] gap-2 text-xs text-surface-500 px-1">
                <span>옵션명</span>
                <span>SKU</span>
                <span className="text-right">가격</span>
                <span className="text-right">재고</span>
                <span className="text-center">활성</span>
                <span></span>
              </div>
              {variants.map((v, idx) => (
                <div
                  key={idx}
                  className="grid grid-cols-[1fr_1fr_120px_80px_60px_30px] gap-2 items-center"
                >
                  <input
                    type="text"
                    value={v.name}
                    onChange={e =>
                      setVariants(arr => {
                        const c = [...arr];
                        c[idx] = { ...c[idx], name: e.target.value };
                        return c;
                      })
                    }
                    placeholder="예: 빨강 / Left arm"
                    className={inputCls}
                  />
                  <input
                    type="text"
                    value={v.sku}
                    onChange={e =>
                      setVariants(arr => {
                        const c = [...arr];
                        c[idx] = { ...c[idx], sku: e.target.value };
                        return c;
                      })
                    }
                    placeholder="leader-piper-red"
                    className={inputCls + " font-mono text-xs"}
                  />
                  <input
                    type="number"
                    min={0}
                    step={1000}
                    value={v.priceKrw}
                    onChange={e =>
                      setVariants(arr => {
                        const c = [...arr];
                        c[idx] = { ...c[idx], priceKrw: Number(e.target.value) || 0 };
                        return c;
                      })
                    }
                    className={inputCls + " font-mono text-right"}
                  />
                  <input
                    type="number"
                    min={0}
                    value={v.stock}
                    onChange={e =>
                      setVariants(arr => {
                        const c = [...arr];
                        c[idx] = { ...c[idx], stock: Number(e.target.value) || 0 };
                        return c;
                      })
                    }
                    className={inputCls + " text-right"}
                  />
                  <label className="flex items-center justify-center cursor-pointer">
                    <input
                      type="checkbox"
                      checked={v.active}
                      onChange={e =>
                        setVariants(arr => {
                          const c = [...arr];
                          c[idx] = { ...c[idx], active: e.target.checked };
                          return c;
                        })
                      }
                      className="w-4 h-4 accent-indigo-500"
                    />
                  </label>
                  <button
                    type="button"
                    onClick={() => setVariants(arr => arr.filter((_, i) => i !== idx))}
                    className="text-surface-500 hover:text-red-400 text-sm"
                    title="삭제"
                  >
                    ✕
                  </button>
                </div>
              ))}
              <button
                type="button"
                onClick={() =>
                  setVariants(arr => [
                    ...arr,
                    { name: "", sku: "", priceKrw, stock: 0, active: true },
                  ])
                }
                className="px-3 py-1.5 rounded-md bg-white/5 border border-white/10 hover:bg-white/10 text-xs text-surface-200"
              >
                + 옵션 추가
              </button>
              <p className="text-xs text-surface-500">
                옵션 사용 시 스토어에서 고객은 옵션을 선택해야 결제할 수 있습니다.
                기본 가격·재고는 옵션의 가격·재고로 대체됩니다.
              </p>
            </div>
          )}
        </div>

        <label className="flex items-center gap-2 text-sm cursor-pointer select-none">
          <input
            type="checkbox"
            className="w-4 h-4 accent-indigo-500"
            checked={active}
            onChange={e => setActive(e.target.checked)}
          />
          <span className="text-surface-200">활성 (스토어에 노출)</span>
        </label>

        {error && (
          <p className="text-xs text-red-400 bg-red-500/10 border border-red-500/20 rounded-md px-3 py-2">
            {error}
          </p>
        )}

        <div className="flex items-center justify-between pt-3 border-t border-white/5">
          <div>
            {isEdit && (
              <button
                type="button"
                onClick={remove}
                disabled={deleting}
                className="text-xs text-red-300 hover:text-red-200 underline disabled:opacity-50"
              >
                {deleting ? "삭제 중…" : "상품 삭제"}
              </button>
            )}
          </div>
          <div className="flex gap-2">
            <button
              type="button"
              onClick={() => router.push("/admin/products")}
              className="px-4 py-2 rounded-full bg-white/5 hover:bg-white/10 border border-white/10 text-xs font-semibold"
            >
              취소
            </button>
            <button
              type="button"
              onClick={save}
              disabled={
                saving ||
                !name ||
                priceKrw < 0 ||
                (!hasVariants && !sku.trim()) ||
                (hasVariants &&
                  !variants.some(v => v.name.trim() && v.sku.trim()))
              }
              className="px-5 py-2 rounded-full bg-gradient-to-r from-indigo-500 to-purple-500 text-white text-xs font-semibold disabled:opacity-40 disabled:cursor-not-allowed"
            >
              {saving ? "저장 중…" : isEdit ? "변경 저장" : "상품 등록"}
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}

const inputCls =
  "w-full px-3 py-2 rounded-lg bg-surface-900/60 border border-white/10 text-surface-100 text-sm focus:outline-none focus:border-indigo-500/60 focus:ring-2 focus:ring-indigo-500/20";

function Field({
  label,
  required,
  hint,
  children,
}: {
  label: string;
  required?: boolean;
  hint?: string;
  children: React.ReactNode;
}) {
  return (
    <label className="block">
      <span className="text-xs font-semibold text-surface-200">
        {label}
        {required && <span className="text-red-400 ml-0.5">*</span>}
        {hint && <span className="text-surface-500 font-normal ml-2">{hint}</span>}
      </span>
      <div className="mt-1.5">{children}</div>
    </label>
  );
}
