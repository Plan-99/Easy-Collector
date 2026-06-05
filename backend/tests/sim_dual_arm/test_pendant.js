/**
 * dual_arm_test — RobotPendant IK verification (Playwright).
 *
 * Per the test plan, RobotPendant (the task-space IK jog UI) is verified
 * through the browser. This script:
 *   1. ensures the dual_arm_test sim is up + the robot is subscribed (REST)
 *   2. opens the operator UI RobotPage, watches the role='dual_arm' robot
 *   3. confirms the pendant shows TWO end-effector columns (L_ee + R_ee)
 *   4. clicks the L_ee 'z' (+) jog button repeatedly and asserts the displayed
 *      L_ee z value changes; repeats for R_ee
 *   5. screenshots the pendant for visual confirmation
 *
 * Run via the playwright-skill executor (resolves the playwright module):
 *   cd .claude/skills/playwright-skill && \
 *     node run.js ../../../backend/tests/sim_dual_arm/test_pendant.js
 *
 * Assumes backend (5000) + frontend dev server (5173) + ROS2 bridge are up.
 */
const { chromium } = require('playwright');

const BACKEND = process.env.EC_BACKEND || 'http://127.0.0.1:5000';
const FRONTEND = process.env.EC_FRONTEND || 'http://localhost:5173';
const ENV = 'dual_arm_test';
const SHOT_DIR = '/tmp';

async function api(method, path, body) {
  const res = await fetch(`${BACKEND}/api${path}`, {
    method,
    headers: { 'Content-Type': 'application/json' },
    body: body ? JSON.stringify(body) : undefined,
  });
  const text = await res.text();
  try { return JSON.parse(text); } catch { return { raw: text }; }
}

async function ensureSim() {
  const start = await api('POST', `/${ENV}:start`, { show_viewer: false });
  if (start.status !== 'success') throw new Error(`${ENV}:start failed: ${JSON.stringify(start)}`);
  const robotId = start.robot_ids.dual_arm;
  // wait for topics
  for (let i = 0; i < 90; i++) {
    const st = await api('GET', `/${ENV}/status`);
    if (st.has_topics) break;
    await new Promise(r => setTimeout(r, 1000));
  }
  await api('POST', `/robot/${robotId}/:subscribe_robot`);
  return robotId;
}

// Read the numeric value cell for an EE axis row inside the pendant.
// ee_col_index: 0 = first EE column (L_ee), label e.g. 'z'.
async function eeRow(page, eeColIndex, axisLabel) {
  // Each watched robot is a q-tab-panel; inactive panels stay mounted but
  // hidden, so scope to :visible to hit only the active robot's pendant.
  // Within it the EE column rows are [remove][label][value][add]; L_ee axes
  // come first, then R_ee. nth(0)=L_ee axis, nth(1)=R_ee axis.
  const rows = page.locator('.row.flex-center:visible', { has: page.locator('div.text-caption', { hasText: new RegExp(`^${axisLabel}$`) }) });
  return rows.nth(eeColIndex);
}

(async () => {
  const robotId = await ensureSim();
  console.log(`[setup] ${ENV} sim ready, robot_id=${robotId}`);

  const browser = await chromium.launch({ headless: true });
  const ctx = await browser.newContext({ viewport: { width: 1680, height: 950 } });
  const page = await ctx.newPage();
  const errors = [];
  page.on('pageerror', e => errors.push('pageerror: ' + e.message));

  const results = [];
  const check = (name, ok, detail = '') => {
    results.push([name, ok]);
    console.log(`  [${ok ? 'PASS' : 'FAIL'}] ${name}  ${detail}`);
  };

  try {
    await page.goto(`${FRONTEND}/#/robots/management`, { waitUntil: 'domcontentloaded', timeout: 30000 });
    await page.waitForLoadState('networkidle', { timeout: 15000 }).catch(() => {});
    await page.waitForTimeout(1500);
    await page.screenshot({ path: `${SHOT_DIR}/${ENV}_robotpage.png` });

    // Watch the dual-arm robot: click its card image (triggers watchRobot).
    const card = page.locator('.q-card', { hasText: 'dual_arm_test_robot' }).first();
    await card.waitFor({ timeout: 15000 });
    await card.locator('img, .q-img').first().click();
    await page.waitForTimeout(2500);  // let pendant dialog open + socket populate eePos
    await page.screenshot({ path: `${SHOT_DIR}/${ENV}_pendant.png` });

    // Expect two EE blocks => L_ee + R_ee => an 'x' axis label appears twice
    // in the visible (active) panel.
    const xLabels = await page.locator('div.text-caption:visible', { hasText: /^x$/ }).count();
    check('pendant shows two EE columns (L_ee + R_ee)', xLabels === 2, `(visible x-rows=${xLabels})`);

    // --- L_ee z jog ---
    for (const [idx, name] of [[0, 'L_ee'], [1, 'R_ee']]) {
      const row = await eeRow(page, idx, 'z');
      const valCell = row.locator('div.text-primary');
      const before = parseFloat((await valCell.textContent()) || '0');
      const addBtn = row.locator('button').last();
      for (let k = 0; k < 25; k++) { await addBtn.click(); await page.waitForTimeout(60); }
      await page.waitForTimeout(800);
      const after = parseFloat((await valCell.textContent()) || '0');
      check(`${name} z jog changes displayed value`, Math.abs(after - before) > 0.002,
        `(${before.toFixed(4)} -> ${after.toFixed(4)})`);
    }

    await page.screenshot({ path: `${SHOT_DIR}/${ENV}_pendant_after.png` });
    if (errors.length) console.log('  [page errors]', errors.slice(0, 5));
  } catch (e) {
    console.error('✗ pendant test error:', e.message);
    await page.screenshot({ path: `${SHOT_DIR}/${ENV}_error.png` }).catch(() => {});
    results.push(['exception', false]);
  } finally {
    await browser.close();
  }

  const passed = results.filter(([, ok]) => ok).length;
  console.log(`\n=== ${ENV} pendant: ${passed}/${results.length} checks passed ===`);
  console.log(`screenshots: ${SHOT_DIR}/${ENV}_*.png`);
  process.exit(passed === results.length && results.length > 0 ? 0 : 1);
})();
