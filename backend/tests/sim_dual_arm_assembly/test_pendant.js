/**
 * dual_arm_assembly_test — RobotPendant IK verification (Playwright).
 *
 * Two role='single_arm' robots (left_arm, right_arm) on separate topics,
 * combined via an Assembly. Each has a single 'ee'. This script watches each
 * robot on the operator RobotPage and verifies its task-space (IK) jog works:
 *   1. ensure sim up + both robots subscribed (REST)
 *   2. for each robot: watch it, jog 'ee' z (+), assert displayed value changes
 *   3. screenshot each pendant
 *
 * Run via the playwright-skill executor:
 *   cd .claude/skills/playwright-skill && \
 *     node run.js ../../../backend/tests/sim_dual_arm_assembly/test_pendant.js
 *
 * Assumes backend (5000) + frontend (5173) + ROS2 bridge are up.
 */
const { chromium } = require('playwright');

const BACKEND = process.env.EC_BACKEND || 'http://127.0.0.1:5000';
const FRONTEND = process.env.EC_FRONTEND || 'http://localhost:5173';
const ENV = 'dual_arm_assembly_test';
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
  for (let i = 0; i < 90; i++) {
    const st = await api('GET', `/${ENV}/status`);
    if (st.has_topics) break;
    await new Promise(r => setTimeout(r, 1000));
  }
  await api('POST', `/robot/${start.robot_ids.left_arm}/:subscribe_robot`);
  await api('POST', `/robot/${start.robot_ids.right_arm}/:subscribe_robot`);
  return start.robot_ids;
}

(async () => {
  const ids = await ensureSim();
  console.log(`[setup] ${ENV} sim ready, robots=${JSON.stringify(ids)}`);

  const browser = await chromium.launch({ headless: true });
  const ctx = await browser.newContext({ viewport: { width: 1680, height: 950 } });
  const page = await ctx.newPage();
  const results = [];
  const check = (name, ok, detail = '') => {
    results.push([name, ok]);
    console.log(`  [${ok ? 'PASS' : 'FAIL'}] ${name}  ${detail}`);
  };

  try {
    await page.goto(`${FRONTEND}/#/robots/management`, { waitUntil: 'domcontentloaded', timeout: 30000 });
    await page.waitForLoadState('networkidle', { timeout: 15000 }).catch(() => {});
    await page.waitForTimeout(1500);

    // RobotPage renders one q-tab per robot; clicking a robot card watches it
    // and activates that tab. First watch each robot via its card image so the
    // pendant mounts, then switch between them by clicking the tab (reliable).
    for (const name of ['dual_arm_assembly_left_arm', 'dual_arm_assembly_right_arm']) {
      const card = page.locator('.q-card', { hasText: name }).first();
      await card.waitFor({ timeout: 15000 });
      // force: a bottom-docked console bar overlaps cards once a robot is watched.
      await card.locator('img, .q-img').first().click({ force: true });
      await page.waitForTimeout(1200);
    }

    for (const name of ['dual_arm_assembly_left_arm', 'dual_arm_assembly_right_arm']) {
      // Activate this robot's tab (label is the robot name, CSS-uppercased).
      await page.locator('.q-tab', { hasText: name }).first().click({ force: true });
      await page.waitForTimeout(2000);
      await page.screenshot({ path: `${SHOT_DIR}/${ENV}_${name}_pendant.png` });

      // single_arm -> one EE column -> 'z' axis row in the visible (active) panel.
      const zRow = page.locator('.row.flex-center:visible', {
        has: page.locator('div.text-caption', { hasText: /^z$/ }),
      }).first();
      const valCell = zRow.locator('div.text-primary');
      const before = parseFloat((await valCell.textContent().catch(() => '0')) || '0');
      const addBtn = zRow.locator('button').last();
      // force: a bottom-docked console bar can overlap lower rows.
      for (let k = 0; k < 25; k++) { await addBtn.click({ force: true }); await page.waitForTimeout(60); }
      await page.waitForTimeout(800);
      const after = parseFloat((await valCell.textContent().catch(() => '0')) || '0');
      check(`${name}: ee z jog changes value`, Math.abs(after - before) > 0.002,
        `(${before.toFixed(4)} -> ${after.toFixed(4)})`);
    }
  } catch (e) {
    console.error('✗ assembly pendant test error:', e.message);
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
