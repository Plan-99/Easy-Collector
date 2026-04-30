import Navbar from "@/components/Navbar";
import ScrollReveal from "@/components/ScrollReveal";
import Footer from "@/components/Footer";
import Link from "next/link";
import { auth } from "@/auth";

export default async function Home() {
  const session = await auth();
  return (
    <>
      <ScrollReveal />

      {/* Floating orbs */}
      <div className="fixed inset-0 -z-10 overflow-hidden pointer-events-none">
        <div className="orb w-[500px] h-[500px] bg-indigo-600/20 -top-40 -left-40 animate-float" />
        <div className="orb w-[400px] h-[400px] bg-purple-600/15 top-1/3 -right-32 animate-float-delayed" />
        <div className="orb w-[300px] h-[300px] bg-blue-600/10 bottom-20 left-1/4 animate-float" />
      </div>

      <Navbar isLoggedIn={!!session?.user} isAdmin={(session?.user as any)?.role === "admin"} />

      {/* ===== HERO ===== */}
      <section className="hero-mesh relative min-h-dvh flex items-center justify-center pt-16" id="hero">
        <div className="max-w-7xl mx-auto px-6 text-center">
          <div className="reveal inline-flex items-center gap-2 px-4 py-1.5 rounded-full bg-indigo-500/10 border border-indigo-500/20 mb-10">
            <span className="text-sm font-medium text-indigo-300">Imitation Learning Platform</span>
          </div>

          <h1 className="reveal font-[var(--font-display)] font-extrabold text-5xl md:text-7xl lg:text-8xl tracking-tight leading-[1.08] mb-8">
            코드 한 줄 없이,<br />
            <span className="bg-gradient-to-r from-indigo-400 via-purple-400 to-pink-400 bg-clip-text text-transparent">
              로봇에게 작업을 가르치세요.
            </span>
          </h1>

          <p className="reveal text-lg md:text-xl text-surface-400 max-w-xl mx-auto mb-12 leading-relaxed">
            로봇 티칭부터 AI 학습, 추론까지 하나의 웹 UI에서 완결합니다.
            엔지니어 없이도, 브라우저만 있으면 됩니다.
          </p>

          <div className="reveal flex flex-col sm:flex-row gap-4 justify-center items-center mb-20">
            <a href="#pricing" className="btn-pill bg-gradient-to-r from-indigo-500 to-purple-500 text-white shadow-lg shadow-indigo-500/25 hover:shadow-indigo-500/40 hover:-translate-y-0.5">
              무료로 시작하기
              <span className="icon-circle bg-white/20">→</span>
            </a>
            <a href="#services" className="btn-pill bg-white/5 border border-white/10 text-surface-300 hover:text-white hover:bg-white/10">
              데모 영상 보기
              <span className="icon-circle bg-white/10">▶</span>
            </a>
          </div>

          <div className="reveal flex items-center justify-center gap-8 md:gap-14">
            <div className="text-center">
              <div className="font-[var(--font-display)] font-extrabold text-3xl md:text-4xl bg-gradient-to-r from-indigo-400 to-purple-400 bg-clip-text text-transparent">12+</div>
              <div className="text-sm text-surface-500 mt-1">지원 로봇 기종</div>
            </div>
            <div className="w-px h-10 bg-surface-800" />
            <div className="text-center">
              <div className="font-[var(--font-display)] font-extrabold text-3xl md:text-4xl bg-gradient-to-r from-purple-400 to-pink-400 bg-clip-text text-transparent">3</div>
              <div className="text-sm text-surface-500 mt-1">AI 정책 지원</div>
            </div>
            <div className="w-px h-10 bg-surface-800" />
            <div className="text-center">
              <div className="font-[var(--font-display)] font-extrabold text-3xl md:text-4xl bg-gradient-to-r from-pink-400 to-rose-400 bg-clip-text text-transparent">28분</div>
              <div className="text-sm text-surface-500 mt-1">첫 학습까지</div>
            </div>
          </div>
        </div>
      </section>


      {/* ===== PROBLEM ===== */}
      <section className="py-24 md:py-32 lg:py-40" id="problem">
        <div className="max-w-7xl mx-auto px-6">
          <div className="reveal text-center mb-16 md:mb-20">
            <p className="text-sm font-semibold text-indigo-400 tracking-wide mb-4">WHY EASY TRAINER</p>
            <h2 className="font-[var(--font-display)] font-extrabold text-3xl md:text-5xl tracking-tight mb-5">로봇 AI, 왜 이렇게 어려운 걸까요?</h2>
            <p className="text-surface-400 text-lg max-w-md mx-auto">현장에서 매일 반복되는 문제들, 저희가 해결합니다.</p>
          </div>

          <div className="grid md:grid-cols-3 gap-6">
            <div className="reveal reveal-delay-1 bezel-card group hover:-translate-y-2 spring-transition">
              <div className="bezel-inner p-8 md:p-10">
                <div className="w-12 h-12 rounded-xl bg-indigo-500/10 flex items-center justify-center mb-6 group-hover:bg-indigo-500/20 spring-transition">
                  <div className="w-5 h-5 rounded-full bg-indigo-400" />
                </div>
                <h3 className="font-bold text-lg mb-3 leading-snug">&ldquo;코드를 짤 줄 알아야 로봇을 가르칠 수 있나요?&rdquo;</h3>
                <p className="text-surface-400 text-[0.94rem] leading-relaxed">ROS 설정, 데이터 파이프라인, 학습 스크립트까지 — 로봇 하나 움직이려면 엔지니어 3명이 몇 주를 써야 합니다.</p>
              </div>
            </div>
            <div className="reveal reveal-delay-2 bezel-card group hover:-translate-y-2 spring-transition">
              <div className="bezel-inner p-8 md:p-10">
                <div className="w-12 h-12 rounded-xl bg-purple-500/10 flex items-center justify-center mb-6 group-hover:bg-purple-500/20 spring-transition">
                  <div className="w-5 h-5 rounded-full bg-purple-400" />
                </div>
                <h3 className="font-bold text-lg mb-3 leading-snug">&ldquo;데이터는 모았는데 학습이 안 됩니다.&rdquo;</h3>
                <p className="text-surface-400 text-[0.94rem] leading-relaxed">카메라 캘리브레이션, 데이터 포맷 변환, 정규화 설정 — 학습 전에 지쳐버립니다.</p>
              </div>
            </div>
            <div className="reveal reveal-delay-3 bezel-card group hover:-translate-y-2 spring-transition">
              <div className="bezel-inner p-8 md:p-10">
                <div className="w-12 h-12 rounded-xl bg-pink-500/10 flex items-center justify-center mb-6 group-hover:bg-pink-500/20 spring-transition">
                  <div className="w-5 h-5 rounded-full bg-pink-400" />
                </div>
                <h3 className="font-bold text-lg mb-3 leading-snug">&ldquo;새 로봇으로 바꾸면 처음부터 다시?&rdquo;</h3>
                <p className="text-surface-400 text-[0.94rem] leading-relaxed">로봇이 바뀔 때마다 드라이버부터 제어 코드까지 전부 새로 작성하는 건 반복 낭비입니다.</p>
              </div>
            </div>
          </div>
        </div>
      </section>

      {/* ===== FEATURES BENTO ===== */}
      <section className="py-24 md:py-32 bg-surface-900/50 border-y border-white/5">
        <div className="max-w-7xl mx-auto px-6">
          <div className="reveal text-center mb-16 md:mb-20">
            <p className="text-sm font-semibold text-purple-400 tracking-wide mb-4">FEATURES</p>
            <h2 className="font-[var(--font-display)] font-extrabold text-3xl md:text-5xl tracking-tight mb-5">Easy Trainer가 해결합니다</h2>
          </div>

          <div className="grid md:grid-cols-5 gap-6">
            <div className="reveal md:col-span-3 bezel-card">
              <div className="bezel-inner p-8 md:p-10 h-full flex flex-col justify-between">
                <div>
                  <div className="flex items-center gap-3 mb-6">
                    <div className="w-10 h-10 rounded-lg bg-indigo-500/15 flex items-center justify-center">
                      <div className="w-4 h-4 rounded bg-indigo-400" />
                    </div>
                    <span className="text-sm font-semibold text-indigo-400">웹 기반 올인원</span>
                  </div>
                  <h3 className="font-[var(--font-display)] font-bold text-2xl mb-3 tracking-tight">브라우저에서 모든 것을 처리합니다</h3>
                  <p className="text-surface-400 leading-relaxed">로봇 연결, 센서 설정, 데이터 수집, 정책 학습, 추론 테스트까지 — 터미널을 열 필요가 없습니다. Docker 한 줄이면 설치 완료.</p>
                </div>
                <div className="mt-8 rounded-xl bg-surface-950/60 border border-white/5 p-4 font-mono text-sm text-surface-400">
                  <span className="text-green-400">$</span> docker compose up -d service<br />
                  <span className="text-surface-600"># localhost:5000 에서 바로 시작</span>
                </div>
              </div>
            </div>
            <div className="reveal reveal-delay-1 md:col-span-2 flex flex-col gap-6">
              <div className="bezel-card flex-1">
                <div className="bezel-inner p-8 h-full">
                  <div className="w-10 h-10 rounded-lg bg-purple-500/15 flex items-center justify-center mb-5">
                    <div className="w-4 h-4 rounded bg-purple-400" />
                  </div>
                  <h3 className="font-bold text-lg mb-2">ACT, Diffusion, PI0</h3>
                  <p className="text-surface-400 text-sm leading-relaxed">최신 모방학습 정책 3종을 UI에서 하이퍼파라미터 조정까지 지원합니다. LoRA, Mixed Precision도 클릭 한 번.</p>
                </div>
              </div>
              <div className="bezel-card flex-1">
                <div className="bezel-inner p-8 h-full">
                  <div className="w-10 h-10 rounded-lg bg-emerald-500/15 flex items-center justify-center mb-5">
                    <div className="w-4 h-4 rounded bg-emerald-400" />
                  </div>
                  <h3 className="font-bold text-lg mb-2">원격 학습 서버</h3>
                  <p className="text-surface-400 text-sm leading-relaxed">GPU 서버를 분리해 데이터 업로드, 원격 학습, 모델 자동 수신까지 한 번에 처리합니다.</p>
                </div>
              </div>
            </div>
          </div>
        </div>
      </section>

      {/* ===== VIDEO ===== */}
      <section className="py-24 md:py-32" id="services">
        <div className="max-w-6xl mx-auto px-6 reveal">
          <div className="relative w-full rounded-2xl overflow-hidden border border-white/5" style={{ paddingBottom: "56.25%" }}>
            <div className="absolute inset-0 z-10" />
            <iframe
              className="absolute inset-0 w-full h-full"
              src="https://www.youtube.com/embed/NG3V1qwlDgE?autoplay=1&mute=1&loop=1&playlist=NG3V1qwlDgE&controls=0&showinfo=0&rel=0&modestbranding=1&iv_load_policy=3&disablekb=1"
              title="Easy Trainer Demo"
              frameBorder="0"
              allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
              allowFullScreen
              loading="lazy"
            />
          </div>
        </div>
      </section>

      {/* ===== TESTIMONIALS ===== */}
      <section className="py-24 md:py-32 bg-surface-900/50 border-y border-white/5" id="testimonials">
        <div className="max-w-7xl mx-auto px-6">
          <div className="reveal text-center mb-16 md:mb-20">
            <p className="text-sm font-semibold text-emerald-400 tracking-wide mb-4">TESTIMONIALS</p>
            <h2 className="font-[var(--font-display)] font-extrabold text-3xl md:text-5xl tracking-tight mb-5">현장의 목소리</h2>
            <p className="text-surface-400 text-lg">Easy Trainer를 도입한 팀들의 경험입니다.</p>
          </div>

          <div className="grid md:grid-cols-2 gap-6 max-w-5xl mx-auto">
            <div className="reveal reveal-delay-1 bezel-card group hover:-translate-y-1 spring-transition">
              <div className="bezel-inner p-8 md:p-10">
                <div className="flex gap-1 mb-6">
                  <span className="text-amber-400 text-lg">★</span>
                  <span className="text-amber-400 text-lg">★</span>
                  <span className="text-amber-400 text-lg">★</span>
                  <span className="text-amber-400 text-lg">★</span>
                  <span className="text-amber-400 text-lg">★</span>
                </div>
                <p className="text-surface-300 leading-relaxed mb-8">&ldquo;이전에는 ACT 학습 파이프라인 하나 세팅하는 데 2주가 걸렸습니다. Easy Trainer 도입 후 같은 작업을 하루 만에 끝냈습니다. 비전공 연구원도 혼자서 데이터 수집부터 추론 테스트까지 할 수 있게 됐습니다.&rdquo;</p>
                <div className="flex items-center gap-4">
                  <div className="w-11 h-11 rounded-full bg-gradient-to-br from-indigo-500 to-purple-500 flex items-center justify-center font-bold text-sm">김</div>
                  <div>
                    <div className="font-semibold text-sm">김OO</div>
                    <div className="text-surface-500 text-sm">기계공학과 연구실 연구원</div>
                  </div>
                </div>
              </div>
            </div>
            <div className="reveal reveal-delay-2 bezel-card group hover:-translate-y-1 spring-transition">
              <div className="bezel-inner p-8 md:p-10">
                <div className="flex gap-1 mb-6">
                  <span className="text-amber-400 text-lg">★</span>
                  <span className="text-amber-400 text-lg">★</span>
                  <span className="text-amber-400 text-lg">★</span>
                  <span className="text-amber-400 text-lg">★</span>
                  <span className="text-amber-400 text-lg">★</span>
                </div>
                <p className="text-surface-300 leading-relaxed mb-8">&ldquo;공장 라인에 Piper, TM12, JAKA를 섞어 쓰고 있는데 Easy Trainer에서 전부 한 화면으로 관리됩니다. 로봇 추가할 때 코드 수정 없이 웹에서 클릭 몇 번이면 끝이라 현장 엔지니어들 반응이 제일 좋았습니다.&rdquo;</p>
                <div className="flex items-center gap-4">
                  <div className="w-11 h-11 rounded-full bg-gradient-to-br from-purple-500 to-pink-500 flex items-center justify-center font-bold text-sm">박</div>
                  <div>
                    <div className="font-semibold text-sm">박OO</div>
                    <div className="text-surface-500 text-sm">제조 AI 솔루션 기업 PM</div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </section>

      {/* ===== PRICING ===== */}
      <section className="py-24 md:py-32 lg:py-40" id="pricing">
        <div className="max-w-7xl mx-auto px-6">
          <div className="reveal text-center mb-16 md:mb-20">
            <p className="text-sm font-semibold text-indigo-400 tracking-wide mb-4">PRICING</p>
            <h2 className="font-[var(--font-display)] font-extrabold text-3xl md:text-5xl tracking-tight mb-5">필요한 만큼만 선택하세요</h2>
            <p className="text-surface-400 text-lg">Free 플랜으로 시작하고, 무제한 플랜으로 여러 PC에서 활용하세요.</p>
          </div>

          <div className="grid md:grid-cols-3 gap-6 max-w-5xl mx-auto">
            {/* Free */}
            <div className="reveal reveal-delay-1 bezel-card group hover:-translate-y-2 spring-transition">
              <div className="bezel-inner p-8 md:p-10 flex flex-col h-full">
                <p className="text-sm font-semibold text-surface-500 uppercase tracking-wider mb-6">Free</p>
                <div className="mb-8">
                  <span className="font-[var(--font-display)] font-extrabold text-5xl tracking-tight">0</span>
                  <span className="text-surface-400 text-sm ml-1">원 / 영구 무료</span>
                </div>
                <ul className="space-y-3 mb-10 flex-1">
                  <li className="flex items-center gap-3 text-surface-300 text-[0.94rem]"><span className="text-emerald-400 flex-shrink-0">✓</span>PC 1대에 등록</li>
                  <li className="flex items-center gap-3 text-surface-300 text-[0.94rem]"><span className="text-emerald-400 flex-shrink-0">✓</span>기본 정책 학습 (ACT)</li>
                  <li className="flex items-center gap-3 text-surface-300 text-[0.94rem]"><span className="text-emerald-400 flex-shrink-0">✓</span>데이터 수집 및 관리</li>
                  <li className="flex items-center gap-3 text-surface-300 text-[0.94rem]"><span className="text-emerald-400 flex-shrink-0">✓</span>커뮤니티 지원</li>
                </ul>
                <Link href="/auth/signin" className="w-full py-3.5 rounded-full bg-white/5 border border-white/10 font-semibold text-sm hover:bg-white/10 spring-transition text-center block">
                  무료로 시작하기
                </Link>
              </div>
            </div>

            {/* Unlimited (recommended) */}
            <div className="reveal reveal-delay-2 relative group hover:-translate-y-2 spring-transition">
              <div className="absolute -inset-px rounded-[21px] bg-gradient-to-b from-indigo-500/40 to-purple-500/20 pointer-events-none" />
              <div className="bezel-card relative">
                <div className="bezel-inner p-8 md:p-10 flex flex-col h-full">
                  <div className="flex items-center gap-3 mb-6">
                    <p className="text-sm font-semibold text-indigo-400 uppercase tracking-wider">Unlimited</p>
                    <span className="px-2.5 py-0.5 rounded-full bg-indigo-500/15 text-indigo-300 text-xs font-semibold">추천</span>
                  </div>
                  <div className="mb-8">
                    <span className="font-[var(--font-display)] font-extrabold text-5xl tracking-tight">출시 기념 무료</span>
                  </div>
                  <ul className="space-y-3 mb-10 flex-1">
                    <li className="flex items-center gap-3 text-surface-300 text-[0.94rem]"><span className="text-emerald-400 flex-shrink-0">✓</span>무제한 PC 등록</li>
                    <li className="flex items-center gap-3 text-surface-300 text-[0.94rem]"><span className="text-emerald-400 flex-shrink-0">✓</span>모든 정책 (ACT, Diffusion, PI0)</li>
                    <li className="flex items-center gap-3 text-surface-300 text-[0.94rem]"><span className="text-emerald-400 flex-shrink-0">✓</span>원격 학습 서버 지원</li>
                    <li className="flex items-center gap-3 text-surface-300 text-[0.94rem]"><span className="text-emerald-400 flex-shrink-0">✓</span>LoRA / Mixed Precision</li>
                    <li className="flex items-center gap-3 text-surface-300 text-[0.94rem]"><span className="text-emerald-400 flex-shrink-0">✓</span>이메일 지원</li>
                  </ul>
                  <Link href="/auth/signin" className="btn-pill justify-center bg-gradient-to-r from-indigo-500 to-purple-500 text-white shadow-lg shadow-indigo-500/20 hover:shadow-indigo-500/35 hover:-translate-y-0.5 text-sm">
                    가입하고 시작하기
                    <span className="icon-circle bg-white/20">→</span>
                  </Link>
                </div>
              </div>
            </div>

            {/* Business */}
            <div className="reveal reveal-delay-3 bezel-card group hover:-translate-y-2 spring-transition">
              <div className="bezel-inner p-8 md:p-10 flex flex-col h-full">
                <p className="text-sm font-semibold text-amber-400 uppercase tracking-wider mb-6">Business</p>
                <div className="mb-8">
                  <span className="font-[var(--font-display)] font-extrabold text-5xl tracking-tight">Custom</span>
                </div>
                <ul className="space-y-3 mb-10 flex-1">
                  <li className="flex items-center gap-3 text-surface-300 text-[0.94rem]"><span className="text-emerald-400 flex-shrink-0">✓</span>Unlimited 플랜의 모든 기능</li>
                  <li className="flex items-center gap-3 text-surface-300 text-[0.94rem]"><span className="text-emerald-400 flex-shrink-0">✓</span>전담 기술 지원</li>
                  <li className="flex items-center gap-3 text-surface-300 text-[0.94rem]"><span className="text-emerald-400 flex-shrink-0">✓</span>온프레미스 배포</li>
                  <li className="flex items-center gap-3 text-surface-300 text-[0.94rem]"><span className="text-emerald-400 flex-shrink-0">✓</span>커스텀 모듈 개발</li>
                  <li className="flex items-center gap-3 text-surface-300 text-[0.94rem]"><span className="text-emerald-400 flex-shrink-0">✓</span>SLA 협의</li>
                </ul>
                <a href="mailto:contact@vertic-ai.com" className="w-full py-3.5 rounded-full bg-white/5 border border-white/10 font-semibold text-sm hover:bg-white/10 spring-transition text-center block">
                  문의하기
                </a>
              </div>
            </div>
          </div>
        </div>
      </section>

      {/* ===== FINAL CTA ===== */}
      <section className="relative py-24 md:py-32 lg:py-40 overflow-hidden" id="cta">
        <div className="absolute inset-0 pointer-events-none">
          <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-[600px] h-[400px] rounded-full bg-indigo-600/10 blur-[100px]" />
        </div>
        <div className="relative max-w-7xl mx-auto px-6 text-center">
          <div className="reveal">
            <h2 className="font-[var(--font-display)] font-extrabold text-4xl md:text-6xl tracking-tight mb-6">
              지금 바로<br />
              <span className="bg-gradient-to-r from-indigo-400 via-purple-400 to-pink-400 bg-clip-text text-transparent">로봇을 가르쳐 보세요.</span>
            </h2>
            <p className="text-surface-400 text-lg md:text-xl max-w-md mx-auto mb-12">설치부터 첫 학습까지 28분. Free 플랜은 영구 무료입니다.</p>
            <div className="flex flex-col sm:flex-row gap-4 justify-center items-center">
              <Link href="/auth/signin" className="btn-pill bg-gradient-to-r from-indigo-500 to-purple-500 text-white shadow-lg shadow-indigo-500/25 hover:shadow-indigo-500/40 hover:-translate-y-0.5">
                무료로 시작하기
                <span className="icon-circle bg-white/20">→</span>
              </Link>
              <Link href="/docs" className="btn-pill bg-white/5 border border-white/10 text-surface-300 hover:text-white hover:bg-white/10">
                Documentation 보기
              </Link>
            </div>
          </div>
        </div>
      </section>

      <Footer />
    </>
  );
}
