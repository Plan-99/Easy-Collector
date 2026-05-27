// Multi-view refactor smoke test
// 목적: 기존 single-view 워크스페이스가 그대로 렌더링되는지 (backward compat).
// 우리가 sensor_ids 에 중복 안 넣고 페이지 들어갈 때 enumerateViews 가 같은
// view_key 를 그대로 sensor_id 로 반환하니 모든 dict lookup 이 fallback 으로
// 정상 동작해야 한다. JS 에러 없으면 통과.
const { chromium } = require('playwright');

const TARGET_URL = 'http://localhost:5173/#/workspace';

(async () => {
  const browser = await chromium.launch({ headless: false, slowMo: 200 });
  const ctx = await browser.newContext({ viewport: { width: 1600, height: 1000 } });
  const page = await ctx.newPage();

  const errors = [];
  page.on('console', (msg) => {
    if (msg.type() === 'error') {
      const t = msg.text();
      if (t.includes('Failed to fetch') || t.includes('WebSocket') || t.includes('404')) return;
      errors.push(t);
    }
  });
  page.on('pageerror', (err) => errors.push(`pageerror: ${err.message}`));

  await page.goto(TARGET_URL, { waitUntil: 'networkidle' });
  await page.waitForTimeout(2000);

  await page.screenshot({ path: '/tmp/mv-01-workspace.png', fullPage: true });

  // Workspace 선택
  const ws = page.locator('input[aria-label*="워크스페이스"], input[aria-label*="Workspace"]').first();
  if (await ws.count()) {
    await ws.click();
    await page.waitForTimeout(800);
    const opt = page.locator('.q-menu .q-item').first();
    if (await opt.count()) {
      await opt.click();
      console.log('→ selected first workspace');
      await page.waitForTimeout(2000);
    }
  }

  await page.screenshot({ path: '/tmp/mv-02-ws-selected.png', fullPage: true });

  // 센서 expansion 열기
  const sensorExpansion = page.locator('.q-expansion-item', { hasText: /센서|Sensor/ }).first();
  if (await sensorExpansion.count()) {
    await sensorExpansion.click();
    await page.waitForTimeout(800);
    await page.screenshot({ path: '/tmp/mv-03-sensor-expansion.png', fullPage: true });
    console.log('→ sensor expansion opened');
  }

  // view chip 들이 보이는지 ("기본" 또는 "main")
  const viewChips = page.locator('.q-chip', { hasText: /기본|main|view/ });
  const chipCount = await viewChips.count();
  console.log(`view chips visible: ${chipCount}`);

  // "+" 버튼 보이는지
  const addBtns = page.locator('button.q-btn:has(.q-icon[role="img"])');
  console.log(`add buttons visible: ${await addBtns.count()}`);

  console.log('\n──────── ERRORS ────────');
  if (errors.length === 0) {
    console.log('✅ No JS errors');
  } else {
    errors.forEach((e) => console.log(`❌ ${e}`));
  }

  await page.waitForTimeout(1500);
  await browser.close();
  if (errors.length > 0) process.exit(1);
})();
