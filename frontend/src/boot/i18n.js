import { defineBoot } from '#q-app/wrappers'
import { createI18n } from 'vue-i18n'
import messages from 'src/i18n'

const SUPPORTED = ['en-US', 'ko-KR']
const STORAGE_KEY = 'easytrainer.locale'

function pickInitialLocale () {
  if (typeof window === 'undefined') return 'en-US'
  const stored = window.localStorage?.getItem(STORAGE_KEY)
  if (stored && SUPPORTED.includes(stored)) return stored
  const nav = window.navigator?.language || ''
  if (nav.toLowerCase().startsWith('ko')) return 'ko-KR'
  return 'en-US'
}

// 싱글톤으로 export — composable / store 등 setup() 외부에서 t()가 필요할 때 사용.
export const i18n = createI18n({
  locale: pickInitialLocale(),
  fallbackLocale: 'en-US',
  globalInjection: true,
  messages,
})

// setup() 외부에서 안전하게 호출 가능한 t. (legacy 모드 기준)
export const t = (...args) => i18n.global.t(...args)

// 사용자 로케일 변경. localStorage에 저장하므로 다음 방문에도 유지된다.
// legacy 모드 (string)와 composition 모드 (Ref) 양쪽을 안전하게 처리.
export function setLocale (locale) {
  if (!SUPPORTED.includes(locale)) return
  const cur = i18n.global.locale
  if (cur && typeof cur === 'object' && 'value' in cur) {
    cur.value = locale
  } else {
    i18n.global.locale = locale
  }
  if (typeof window !== 'undefined') {
    window.localStorage?.setItem(STORAGE_KEY, locale)
  }
}

// setup() 외부에서 현재 로케일 읽기.
export function getLocale () {
  const cur = i18n.global.locale
  return cur && typeof cur === 'object' && 'value' in cur ? cur.value : cur
}

export const SUPPORTED_LOCALES = SUPPORTED

export default defineBoot(({ app }) => {
  app.use(i18n)
})
