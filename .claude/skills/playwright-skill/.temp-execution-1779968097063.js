// Verify the new "Replay Episode" planner block end-to-end via UI.
const { chromium } = require('playwright');
const TARGET = 'http://localhost:5173/#/planner';

(async () => {
  const browser = await chromium.launch({ headless: false, slowMo: 250 });
  const ctx = await browser.newContext({ viewport: { width: 1600, height: 1000 } });
  const page = await ctx.newPage();

  const errors = [];
  page.on('console', m => {
    const t = m.text();
    if (m.type() === 'error') {
      if (t.includes('WebSocket') || t.includes('404')) return;
      errors.push(`[error] ${t}`);
      console.log(`  [console.error] ${t.slice(0, 300)}`);
    } else if (m.type() === 'warning') {
      if (!t.includes('[vite]') && !t.includes('Process store') && !t.includes('Modules store') && !t.includes('Topic store')) {
        console.log(`  [console.warning] ${t.slice(0, 300)}`);
      }
    }
  });
  page.on('pageerror', e => {
    errors.push(`pageerror: ${e.message}`);
    console.log(`  [pageerror] ${e.message.slice(0, 300)}`);
  });

  // log all API responses + all console msgs
  page.on('response', async (res) => {
    const url = res.url();
    if (url.includes('/api/')) {
      console.log(`  ← ${res.status()} ${url.replace('http://localhost:5000', '')}`);
    }
  });
  page.on('console', (m) => {
    if (m.type() === 'log' || m.type() === 'warning') {
      const t = m.text();
      if (t.length < 200 && !t.includes('[vite]') && !t.includes('Process store') && !t.includes('Modules store') && !t.includes('Topic store')) {
        console.log(`  [console.${m.type()}] ${t}`);
      }
    }
  });

  console.log('=== Step 1: Navigate to /planner ===');
  await page.goto(TARGET, { waitUntil: 'domcontentloaded' });
  // Poll for /api/tasks call.
  let tasksSeen = false;
  page.on('response', (r) => { if (r.url().endsWith('/api/tasks')) tasksSeen = true; });
  for (let i = 0; i < 30; i++) {
    await page.waitForTimeout(1000);
    const disabled = await page.locator('.q-field--disabled').count();
    if (disabled === 0) {
      console.log(`  ✅ pageLoading cleared after ~${i+1}s (tasks API seen: ${tasksSeen})`);
      break;
    }
    if (i === 9 || i === 19) {
      console.log(`  ...still loading after ${i+1}s (tasks API seen: ${tasksSeen})`);
    }
  }
  await page.screenshot({ path: '/tmp/planner-01.png', fullPage: true });

  console.log('\n=== Step 2: Select planner "cable" ===');
  // Wait longer for planner page to load
  await page.waitForTimeout(2000);
  // First q-select on page is the language picker. The planner picker has the
  // label/placeholder "플래너 선택" inside the page body (not in the toolbar).
  // Filter by visible text.
  const allSelects = await page.locator('.q-select').count();
  console.log(`  total q-select on page: ${allSelects}`);
  const plannerSelect = page.locator('.q-select').filter({ hasText: '플래너 선택' }).first();
  console.log(`  matching "플래너 선택": ${await plannerSelect.count()}`);
  if (await plannerSelect.count()) {
    await plannerSelect.click();
    await page.waitForTimeout(1500);
    await page.screenshot({ path: '/tmp/planner-02a-dropdown-open.png', fullPage: true });
    const menuItems = await page.locator('.q-menu .q-item').count();
    const virtualItems = await page.locator('.q-virtual-scroll__padding').count();
    const allItems = await page.locator('.q-item').count();
    console.log(`  dropdown — .q-menu .q-item: ${menuItems}, .q-virtual-scroll: ${virtualItems}, all .q-item: ${allItems}`);
    if (menuItems > 0) {
      const all = (await page.locator('.q-menu .q-item').allTextContents()).map(s => s.trim());
      console.log(`  available planners: ${JSON.stringify(all)}`);
      const cableIdx = all.findIndex(s => s.toLowerCase() === 'cable');
      if (cableIdx >= 0) {
        await page.locator('.q-menu .q-item').nth(cableIdx).click();
        console.log(`  ✅ selected: cable (index ${cableIdx})`);
      } else {
        await page.locator('.q-menu .q-item').first().click();
        console.log(`  ✅ selected first: ${all[0]}`);
      }
      await page.waitForTimeout(3000);
    } else {
      console.log('  ❌ dropdown empty');
    }
  } else {
    console.log('  ❌ no q-select on page');
  }
  await page.screenshot({ path: '/tmp/planner-02-selected.png', fullPage: true });

  // After selecting a planner, the workspace area + groups should appear.
  console.log('\n=== Step 3: Find a group, click add-block (+) ===');
  // Try to find the "+" / 블록 추가 button — usually 'add' icon button or text
  const addBlockText = page.locator('button', { hasText: /블록 추가|Add Block|블록/ }).first();
  let opened = false;
  if (await addBlockText.count()) {
    await addBlockText.click();
    console.log('  ✅ clicked "블록 추가"');
    opened = true;
  } else {
    // Fallback: any q-btn with add icon
    const plus = page.locator('.q-btn').filter({ has: page.locator('.q-icon').filter({ hasText: 'add' }) }).first();
    if (await plus.count()) {
      await plus.click();
      console.log('  ✅ clicked plus icon button');
      opened = true;
    } else {
      console.log('  ❌ no add-block trigger found');
    }
  }
  await page.waitForTimeout(1500);
  await page.screenshot({ path: '/tmp/planner-03-add-block.png', fullPage: true });

  if (!opened) {
    console.log('\n[diagnostic] available buttons:');
    const btns = await page.locator('.q-btn').all();
    for (let i = 0; i < Math.min(btns.length, 20); i++) {
      const t = await btns[i].textContent();
      if (t && t.trim()) console.log(`  - ${t.trim().slice(0, 60)}`);
    }
  }

  console.log('\n=== Step 4: Open block-type dropdown and pick "Replay Episode" ===');
  // The block-type selector label is "블록 종류" (plannerBlockTypeLabel = '블록 종류')
  const blockTypeField = page.locator('.q-field').filter({ hasText: '블록 종류' }).first();
  if (await blockTypeField.count()) {
    await blockTypeField.click();
    await page.waitForTimeout(700);
    // i18n label is "Replay Episode" (backend label) — Korean translation in i18n
    // but block_configs returns the English "Replay Episode" as label.
    const replayOpt = page.locator('.q-menu .q-item').filter({ hasText: /Replay Episode/ }).first();
    if (await replayOpt.count()) {
      await replayOpt.click();
      console.log('  ✅ picked: Replay Episode');
    } else {
      console.log('  ❌ "Replay Episode" not in dropdown');
      const opts = await page.locator('.q-menu .q-item').allTextContents();
      console.log('  options:', opts.map(s => s.trim()).filter(Boolean));
    }
    await page.waitForTimeout(1500);
  } else {
    console.log('  ❌ block-type select not found');
  }
  await page.screenshot({ path: '/tmp/planner-04-replay-type.png', fullPage: true });

  console.log('\n=== Step 5: Verify the replay-episode form fields ===');
  const labels = [
    ['데이터셋', 'plannerReplayDatasetLabel'],
    ['에피소드', 'plannerReplayEpisodeLabel'],
    ['추론 Hz', 'plannerHzLabel'],
    ['첫 프레임 자세로 먼저 이동', 'plannerReplayMoveToFirst'],
  ];
  for (const [lbl, key] of labels) {
    const cnt = await page.locator(`text=${lbl}`).count();
    console.log(`  ${cnt > 0 ? '✅' : '❌'} ${key}: "${lbl}" (${cnt})`);
  }

  console.log('\n=== Step 6: Open dataset dropdown ===');
  const datasetField = page.locator('.q-field').filter({ hasText: /데이터셋/ }).first();
  if (await datasetField.count()) {
    await datasetField.click();
    await page.waitForTimeout(800);
    const dCount = await page.locator('.q-menu .q-item').count();
    console.log(`  dataset options found: ${dCount}`);
    if (dCount > 0) {
      const opts = (await page.locator('.q-menu .q-item').allTextContents()).map(s => s.trim()).slice(0, 5);
      console.log(`  first few: ${JSON.stringify(opts)}`);
      await page.locator('.q-menu .q-item').first().click();
      console.log('  ✅ picked first dataset');
      await page.waitForTimeout(1200);
    }
  }
  await page.screenshot({ path: '/tmp/planner-05-dataset.png', fullPage: true });

  console.log('\n=== Step 7: Open episode dropdown ===');
  const epField = page.locator('.q-field').filter({ hasText: /^에피소드$/ }).first();
  if (await epField.count()) {
    await epField.click();
    await page.waitForTimeout(800);
    const eCount = await page.locator('.q-menu .q-item').count();
    console.log(`  episode options found: ${eCount}`);
    if (eCount > 0) {
      const opts = (await page.locator('.q-menu .q-item').allTextContents()).map(s => s.trim()).slice(0, 5);
      console.log(`  first few: ${JSON.stringify(opts)}`);
      await page.locator('.q-menu .q-item').first().click();
      console.log('  ✅ picked first episode');
      await page.waitForTimeout(800);
    }
  } else {
    console.log('  ❌ episode dropdown not found');
  }
  await page.screenshot({ path: '/tmp/planner-06-episode.png', fullPage: true });

  console.log('\n=== Step 8: Click Save to add the block ===');
  const saveBtn = page.locator('button').filter({ hasText: /^저장$|^Save$/ }).last();
  if (await saveBtn.count()) {
    await saveBtn.click();
    console.log('  ✅ clicked save');
    await page.waitForTimeout(2000);
  }
  await page.screenshot({ path: '/tmp/planner-07-saved.png', fullPage: true });

  console.log('\n──────── JS errors ────────');
  if (errors.length === 0) console.log('✅ no JS errors');
  else errors.forEach(e => console.log(`❌ ${e}`));

  await page.waitForTimeout(1500);
  await browser.close();
})();
