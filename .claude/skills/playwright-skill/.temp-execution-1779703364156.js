// Verify docs pages actually render the new screenshots
const { chromium } = require('playwright')

const URLS = [
  'http://localhost:3000/docs/planner/create-and-workspaces',
  'http://localhost:3000/docs/planner/blocks-and-run',
  'http://localhost:3000/docs/planner/export',
]

;(async () => {
  const browser = await chromium.launch({ headless: false })
  const ctx = await browser.newContext({ viewport: { width: 1600, height: 900 } })
  const page = await ctx.newPage()

  for (const url of URLS) {
    console.log('=== ', url)
    const resp = await page.goto(url, { waitUntil: 'networkidle', timeout: 30000 }).catch((e) => null)
    console.log('  status:', resp ? resp.status() : 'ERR')
    await page.waitForTimeout(1500)
    // Check for "스크린샷 누락" placeholders
    const missingCount = await page.locator('text=스크린샷 누락').count()
    console.log('  missing-placeholders:', missingCount)
    // Check for img tags with our new files
    const ourImgs = ['create-03-form', 'create-04-edit-delete', 'blocks-03-name', 'blocks-04-save', 'export-01-button']
    for (const slug of ourImgs) {
      const cnt = await page.locator(`img[src*="${slug}"]`).count()
      if (cnt > 0) console.log(`  ✓ ${slug} rendered (${cnt})`)
    }
    // Screenshot the page
    const name = url.split('/').pop()
    await page.screenshot({ path: `/tmp/docs-render-${name}.png`, fullPage: true })
    console.log(`  → /tmp/docs-render-${name}.png`)
  }
  await browser.close()
})().catch((e) => { console.error(e); process.exit(1) })
