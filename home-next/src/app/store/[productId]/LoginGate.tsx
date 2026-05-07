"use client";

import { useState } from "react";
import LoginModal from "@/components/LoginModal";

export default function LoginGate({
  callbackUrl,
  autoOpen = false,
}: {
  callbackUrl: string;
  autoOpen?: boolean;
}) {
  const [open, setOpen] = useState(autoOpen);

  return (
    <>
      <div className="bezel-card mt-4">
        <div className="bezel-inner p-8 text-center space-y-3">
          <p className="font-bold text-surface-100">로그인 후 결제 가능합니다</p>
          <p className="text-sm text-surface-400">
            Easy Trainer 계정으로 로그인하시면 배송지 입력과 결제를 진행할 수 있습니다.
          </p>
          <button
            type="button"
            onClick={() => setOpen(true)}
            className="px-6 py-3 rounded-full bg-gradient-to-r from-indigo-500 to-purple-500 text-white text-sm font-semibold cursor-pointer"
          >
            로그인하고 결제하기
          </button>
        </div>
      </div>
      <LoginModal
        open={open}
        onClose={() => setOpen(false)}
        callbackUrl={callbackUrl}
      />
    </>
  );
}
