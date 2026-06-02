const { chromium } = require('playwright');
(async () => {
  const browser = await chromium.launch({ headless: false });
  const ctx = await browser.newContext({ viewport: { width: 1700, height: 950 } });
  const page = await ctx.newPage();
  const errors = [];
  page.on('pageerror', (e) => errors.push('page: ' + e.message));
  page.on('console', (msg) => {
    if (msg.type() === 'error') errors.push('console: ' + msg.text());
  });
  // 네트워크 로그
  page.on('response', (resp) => {
    const u = resp.url();
    if (u.includes('/api/tasks')) console.log('[net]', resp.status(), u);
  });

  await page.goto('http://localhost:5173/#/workspace', { waitUntil: 'networkidle', timeout: 20000 });
  await page.waitForTimeout(1500);

  const sel = page.locator('.q-select').first();
  await sel.click();
  await page.waitForTimeout(400);
  // Tutorial 클릭
  const opt = page.getByText('Tutorial', { exact: true }).first();
  await opt.click();
  await page.waitForTimeout(3000);

  await page.screenshot({ path: '/tmp/ws-after-fix.png' });

  // 센서/로봇 카운트 확인
  const sensorChips = await page.locator('text=/센서 설정 \\(\\d+\\)/').first().textContent().catch(()=>null);
  console.log('Sensor section:', sensorChips);

  if (errors.length) {
    console.log('--- ERRORS ---'); errors.forEach(e => console.log(e));
  } else { console.log('No JS errors'); }
  await browser.close();
})();
