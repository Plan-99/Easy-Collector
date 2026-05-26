// Second pass: select a workspace, click Inference tab, verify the surrounding
// UI compiles and no JS errors. The actual DAgger button only shows during
// status='testing' which requires a running inference; the dialog template
// is verified by ensuring page renders error-free with the bundle.
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

  // Click workspace selector dropdown
  const ws = page.locator('input[aria-label*="워크스페이스"], input[aria-label*="Workspace"]').first();
  if (await ws.count()) {
    await ws.click();
    await page.waitForTimeout(800);
    // Pick first option in the menu
    const opt = page.locator('.q-menu .q-item').first();
    if (await opt.count()) {
      await opt.click();
      console.log('→ selected first workspace');
      await page.waitForTimeout(1500);
    }
  }

  await page.screenshot({ path: '/tmp/dagger-03-workspace-selected.png', fullPage: true });
  console.log('📸 /tmp/dagger-03-workspace-selected.png');

  // Click Inference tab (Korean: 추론)
  const inferenceTab = page.locator('div[role="tab"]', { hasText: /추론|Inference/ }).first();
  if (await inferenceTab.count()) {
    await inferenceTab.click();
    console.log('→ clicked Inference tab');
    await page.waitForTimeout(1500);
    await page.screenshot({ path: '/tmp/dagger-04-inference-tab.png', fullPage: true });
    console.log('📸 /tmp/dagger-04-inference-tab.png');
  } else {
    console.log('⚠ Inference tab not found');
  }

  console.log('\n──────── ERRORS ────────');
  if (errors.length === 0) {
    console.log('✅ No JS errors');
  } else {
    errors.forEach((e) => console.log(`❌ ${e}`));
  }

  await page.waitForTimeout(1000);
  await browser.close();
  if (errors.length > 0) process.exit(1);
})();
