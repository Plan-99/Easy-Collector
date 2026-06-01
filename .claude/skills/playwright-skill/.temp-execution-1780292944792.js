const { chromium } = require('playwright');
const TARGET_URL = 'http://localhost:5173/#/curriculum';

(async () => {
  const browser = await chromium.launch({ headless: false });
  const context = await browser.newContext({ viewport: { width: 1600, height: 900 } });
  const page = await context.newPage();
  const errors = [];
  page.on('pageerror', (e) => errors.push('pageerror: ' + e.message));
  page.on('console', (msg) => {
    if (msg.type() === 'error') errors.push('console: ' + msg.text());
  });
  try {
    await page.goto(TARGET_URL, { waitUntil: 'domcontentloaded', timeout: 20000 });
    await page.waitForLoadState('networkidle', { timeout: 15000 }).catch(() => {});
    await page.waitForTimeout(2000);
    await page.screenshot({ path: '/tmp/curr-page.png', fullPage: false });
    console.log('Screenshot saved');
    if (errors.length) {
      console.log('--- ERRORS ---');
      errors.forEach((e) => console.log(e));
    } else {
      console.log('No JS errors detected');
    }
  } catch (e) {
    console.error('Load failed:', e.message);
  } finally {
    await browser.close();
  }
})();
