// Lazy loader for the Daum/Kakao Postcode service (다음 우편번호).
// Free, no API key required. Loaded on first use only.
//
//   const result = await openPostcode();
//   if (result) {
//     setPostalCode(result.zonecode);
//     setAddressLine1(result.roadAddress);
//   }

const SCRIPT_URL =
  "https://t1.daumcdn.net/mapjsapi/bundle/postcode/prod/postcode.v2.js";

let scriptPromise: Promise<void> | null = null;

function loadScript(): Promise<void> {
  if (typeof window === "undefined") {
    return Promise.reject(new Error("postcode: server-side call"));
  }
  if (scriptPromise) return scriptPromise;
  type W = Window & { daum?: { Postcode?: unknown } };
  if ((window as W).daum?.Postcode) return Promise.resolve();
  scriptPromise = new Promise<void>((resolve, reject) => {
    const s = document.createElement("script");
    s.src = SCRIPT_URL;
    s.async = true;
    s.onload = () => resolve();
    s.onerror = () => {
      scriptPromise = null;
      reject(new Error("postcode: script load failed"));
    };
    document.head.appendChild(s);
  });
  return scriptPromise;
}

export type PostcodeResult = {
  zonecode: string;
  roadAddress: string;
  jibunAddress: string;
  buildingName?: string;
};

type DaumPostcodeData = {
  zonecode: string;
  address: string;
  roadAddress?: string;
  jibunAddress?: string;
  buildingName?: string;
  bname?: string;
};

type DaumPostcodeCtor = new (opts: {
  oncomplete: (data: DaumPostcodeData) => void;
  onclose?: (state: string) => void;
}) => { open: () => void };

export async function openPostcode(): Promise<PostcodeResult | null> {
  await loadScript();
  return new Promise(resolve => {
    let resolved = false;
    type W = Window & { daum?: { Postcode?: DaumPostcodeCtor } };
    const Postcode = (window as W).daum?.Postcode;
    if (!Postcode) {
      resolve(null);
      return;
    }
    const inst = new Postcode({
      oncomplete: (data: DaumPostcodeData) => {
        resolved = true;
        resolve({
          zonecode: data.zonecode,
          roadAddress: data.roadAddress || data.address,
          jibunAddress: data.jibunAddress || data.address,
          buildingName: data.buildingName || undefined,
        });
      },
      onclose: () => {
        if (!resolved) resolve(null);
      },
    });
    inst.open();
  });
}
