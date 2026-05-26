const { chromium } = require('playwright')

const URLS = [
  'http://localhost:3000/docs/datasets',
  'http://localhost:3000/docs/datasets/edit',
  'http://localhost:3000/docs/datasets/batch-edit',
  'http://localhost:3000/docs/datasets/episode-edit',
  'http://localhost:3000/docs/datasets/import-export',
  'http://localhost:3000/docs/datasets/augment',
  'http://localhost:3000/docs/datasets/downsample',
]

;(async () => {
  const browser = await chromium.launch({ headless: false })
  const ctx = await browser.newContext({ viewport: { width: 1600, height: 900 } })
  const page = await ctx.newPage()
  for (const url of URLS) {
    const resp = await page.goto(url, { waitUntil: 'networkidle', timeout: 30000 }).catch(() => null)
    console.log('===', url, 'status:', resp ? resp.status() : 'ERR')
    await page.waitForTimeout(1500)
    const missingCount = await page.locator('text=스크린샷 누락').count()
    const imgs = await page.locator('img[src*="/docs/datasets/"]').count()
    console.log(`  missing-placeholders: ${missingCount}, imgs: ${imgs}`)
    const name = url.split('/').pop() || 'index'
    await page.screenshot({ path: `/tmp/docs-ds-${name}.png`, fullPage: false })
  }
  await browser.close()
})().catch((e) => { console.error(e); process.exit(1) })
