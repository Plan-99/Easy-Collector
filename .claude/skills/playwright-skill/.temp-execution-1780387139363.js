const { chromium } = require('playwright');
const URL = process.env.URL || 'http://localhost:5173';

(async () => {
  const browser = await chromium.launch({ headless: true });
  const page = await browser.newPage({ viewport: { width: 1680, height: 950 } });
  const consoleErrors = [], pageErrors = [];
  page.on('console', (m) => { if (m.type() === 'error') consoleErrors.push(m.text()); });
  page.on('pageerror', (e) => pageErrors.push(e.message));
  const log = (...a) => console.log(...a);
  const results = [];
  const check = (n, c, e = '') => { results.push({ n, c: !!c }); log(`${c ? 'PASS' : 'FAIL'} — ${n}${e ? ' :: ' + e : ''}`); };

  const pickSelect = async (labelText, optionText) => {
    await page.locator('.q-field', { hasText: labelText }).first().click();
    await page.waitForTimeout(500);
    // 옵션은 방금 열린 드롭다운(.q-menu) 안에서만 클릭 — 뒤 패널의 동명 텍스트 회피
    await page.locator('.q-menu').getByText(optionText, { exact: true }).first().click();
    await page.waitForTimeout(800);
  };

  try {
    await page.goto(`${URL}/#/planner`, { waitUntil: 'networkidle' });
    await page.waitForTimeout(1800);

    // 플래너 선택: cable (id 13, workspace 9/12/17)
    await pickSelect('플래너 선택', 'cable');
    await page.waitForTimeout(1500);

    // 그룹의 "새 블록" + 버튼(round flat icon) 클릭 → 블록 폼.
    // "워크스페이스 추가"(텍스트 버튼)와 구분하기 위해 round 버튼만 타겟.
    await page.locator('.q-btn--round:has(i:text-is("add"))').first().click({ force: true });
    await page.waitForTimeout(1000);
    check('블록 폼 열림', await page.locator('.q-dialog', { hasText: '블록 종류' }).count() > 0);

    // 블록 종류 → Checkpoint
    await pickSelect('블록 종류', 'Checkpoint');
    check('체크포인트 타입 선택됨', await page.getByText('체크포인트 선택').count() > 0 || await page.locator('.q-field', { hasText: '워크스페이스' }).count() > 0);

    // 워크스페이스 → insert_punch_v2 (task 17)
    const wsField = page.locator('.q-field', { hasText: '워크스페이스' });
    if (await wsField.count() > 0) {
      await pickSelect('워크스페이스', 'insert_punch_v2');
    }
    await page.waitForTimeout(600);

    // "체크포인트 선택" 버튼 클릭 → 그래프 다이얼로그
    const selBtn = page.getByRole('button', { name: '체크포인트 선택' });
    check('체크포인트 선택 버튼 존재', await selBtn.count() > 0, `btn=${await selBtn.count()}`);
    await selBtn.first().click({ force: true });
    await page.waitForTimeout(1500);

    const vf = await page.locator('.vue-flow').count();
    const nodes = await page.locator('.vue-flow__node').count();
    check('picker 그래프 렌더', vf > 0 && nodes > 0, `vue-flow=${vf} nodes=${nodes}`);
    await page.screenshot({ path: '/tmp/cp-planner-dialog.png', fullPage: false });

    // 노드 클릭 → 선택 + 다이얼로그 닫힘 + 버튼 라벨이 #id
    if (nodes > 0) {
      await page.locator('.vue-flow__node .q-card').first().click({ force: true });
      await page.waitForTimeout(1000);
      const dialogStillOpen = await page.locator('.vue-flow').count();
      const btnLabel = (await page.getByRole('button', { name: /#\d+/ }).count());
      check('노드 클릭 → 다이얼로그 닫힘', dialogStillOpen === 0, `vue-flow now=${dialogStillOpen}`);
      check('버튼 라벨이 #id 로 갱신', btnLabel > 0, `#id buttons=${btnLabel}`);
      await page.screenshot({ path: '/tmp/cp-planner-selected.png', fullPage: false });
    }
  } catch (e) {
    log('SCRIPT-ERROR:', e.message);
    await page.screenshot({ path: '/tmp/cp-planner-err.png' }).catch(() => {});
  }
  log('\nconsole errors:', consoleErrors.length, '| page errors:', pageErrors.length);
  consoleErrors.slice(0, 8).forEach((e) => log('  CE:', e.slice(0, 160)));
  pageErrors.slice(0, 8).forEach((e) => log('  PE:', e.slice(0, 160)));
  log('SUMMARY:', results.filter(r => r.c).length + '/' + results.length, 'passed');
  await browser.close();
})();
