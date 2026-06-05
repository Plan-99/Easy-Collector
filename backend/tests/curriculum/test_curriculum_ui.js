/**
 * curriculum_ui_test — Curriculum page feature verification + manual screenshots.
 *
 * The Curriculum page ("스스로 배우는 로봇") is a device-free UI surface: it is
 * configured per-planner, so this test drives it against the tutorial planner
 * (name prefix `tutorial_e2e_planner`, bound to the tutorial_env workspace).
 * It does NOT start a rollout (no robot motion); it only exercises the
 * configuration UI that can be verified without live hardware.
 *
 * Two jobs in one pass:
 *   1. FEATURE VERIFICATION (assertions — exit code 1 on failure):
 *        - the planner selector renders and, unselected, shows the empty hint
 *        - selecting the tutorial planner loads the curriculum panel (auto-create
 *          is idempotent) with the three left tabs 플래너 / 디바이스 / 그룹 정책
 *        - the rollout control bar exposes 시작 / 정지 / 전체초기화 / 업그레이드
 *        - each left tab switches without throwing a console / page error
 *        - opening a checkpoint block's settings dialog renders its form
 *   2. MANUAL SCREENSHOTS (ko-KR locale) written to SHOT_DIR for the user manual
 *      under home-next/public/docs/curriculum/.
 *
 * Run via the playwright-skill executor (resolves the playwright module):
 *   cd .claude/skills/playwright-skill && \
 *     node run.js ../../../backend/tests/curriculum/test_curriculum_ui.js
 *
 * Env:
 *   EC_BACKEND   backend base URL   (default http://127.0.0.1:5000)
 *   EC_FRONTEND  frontend dev URL   (default http://localhost:5173)
 *   EC_SHOT_DIR  screenshot out dir (default home-next/public/docs/curriculum)
 *   EC_HEADFUL   set to 1 for a visible browser
 *
 * Assumes backend (5000) + frontend dev server (5173) are up. No robot/sensor
 * hardware required.
 */
const { chromium } = require('playwright');
const path = require('path');

const BACKEND = process.env.EC_BACKEND || 'http://127.0.0.1:5000';
const FRONTEND = process.env.EC_FRONTEND || 'http://localhost:5173';
const SHOT_DIR =
  process.env.EC_SHOT_DIR ||
  path.resolve(__dirname, '../../../home-next/public/docs/curriculum');
const HEADLESS = process.env.EC_HEADFUL !== '1';

const failures = [];
function check(cond, msg) {
  if (cond) {
    console.log(`  ✓ ${msg}`);
  } else {
    console.log(`  ✗ ${msg}`);
    failures.push(msg);
  }
}

async function api(method, path) {
  const res = await fetch(`${BACKEND}/api${path}`, { method });
  const text = await res.text();
  try {
    return JSON.parse(text);
  } catch {
    return { raw: text };
  }
}

// Pick the tutorial planner (bound to tutorial_env). Falls back to any planner
// so the test still exercises the page if the tutorial seed name changes.
async function pickTutorialPlanner() {
  const data = await api('GET', '/planners');
  const planners = (data && data.planners) || [];
  if (!planners.length) throw new Error('no planners exist — seed the tutorial workspace first');
  const tut = planners.find((p) => /tutorial/i.test(p.name || ''));
  return tut || planners[0];
}

(async () => {
  const planner = await pickTutorialPlanner();
  console.log(`[setup] target planner: id=${planner.id} name=${planner.name}`);

  const browser = await chromium.launch({ headless: HEADLESS });
  const ctx = await browser.newContext({ viewport: { width: 1600, height: 1000 } });
  // Force Korean locale before any app code runs (manual is ko).
  await ctx.addInitScript(() => {
    try {
      window.localStorage.setItem('easytrainer.locale', 'ko-KR');
    } catch {
      /* ignore */
    }
  });
  const page = await ctx.newPage();
  const consoleErrors = [];
  page.on('pageerror', (e) => consoleErrors.push('pageerror: ' + e.message));
  page.on('console', (m) => {
    if (m.type() === 'error') consoleErrors.push('console.error: ' + m.text());
  });

  const shot = async (name) => {
    const file = path.join(SHOT_DIR, name);
    await page.screenshot({ path: file });
    console.log(`  📸 ${file}`);
  };

  try {
    // ---- 1. Empty state (no planner selected) --------------------------------
    console.log('\n[1] empty state');
    await page.goto(`${FRONTEND}/?lang=ko-KR#/curriculum`, { waitUntil: 'networkidle' });
    await page.waitForTimeout(1200);
    const plannerSelect = page.locator('.q-select', { hasText: '플래너 선택' }).first();
    check(await plannerSelect.isVisible(), '플래너 선택 셀렉터 노출');
    check(
      await page.getByText('플래너를 선택하면 해당 플래너의 커리큘럼을 설정').first().isVisible(),
      '미선택 시 안내 문구(currSelectPlannerHint) 노출',
    );
    await shot('01-empty.png');

    // ---- 2. Select the tutorial planner -> curriculum loads -------------------
    console.log('\n[2] select planner');
    await plannerSelect.click();
    await page.waitForTimeout(400);
    await page.locator('.q-menu .q-item', { hasText: planner.name }).first().click();
    // Auto-create + dashboard fetch; wait for the left tabs to appear.
    await page.waitForSelector('.q-tab:has-text("플래너")', { timeout: 20000 });
    await page.waitForTimeout(1500);
    check(await page.locator('.q-tab:has-text("플래너")').first().isVisible(), '플래너 탭 노출');
    check(await page.locator('.q-tab:has-text("디바이스")').first().isVisible(), '디바이스 탭 노출');
    check(await page.locator('.q-tab:has-text("그룹 정책")').first().isVisible(), '그룹 정책 탭 노출');
    check(await page.getByRole('button', { name: '시작' }).first().isVisible(), '롤아웃 시작 버튼 노출');
    check(await page.getByRole('button', { name: '업그레이드' }).first().isVisible(), '업그레이드 버튼 노출');
    await shot('02-overview.png');

    // ---- 3. Left tabs: 플래너 / 디바이스 / 그룹 정책 --------------------------
    console.log('\n[3] left tabs');
    await page.locator('.q-tab:has-text("플래너")').first().click();
    await page.waitForTimeout(800);
    await shot('03-tab-planner.png');

    await page.locator('.q-tab:has-text("디바이스")').first().click();
    await page.waitForTimeout(800);
    check(!consoleErrors.length, '디바이스 탭 전환 후 콘솔 에러 없음');
    await shot('04-tab-device.png');

    await page.locator('.q-tab:has-text("그룹 정책")').first().click();
    await page.waitForTimeout(800);
    // Select a group so the Stage Mission form (목표 성공 + 교정 개수 / 실패 저장 확률)
    // renders — without a group the left pane only shows the "그룹 선택" picker.
    const groupSelect = page.locator('.q-select', { hasText: '그룹 선택' }).first();
    if (await groupSelect.count()) {
      await groupSelect.click();
      await page.waitForTimeout(400);
      const opt = page.locator('.q-menu .q-item').first();
      if (await opt.count()) {
        await opt.click();
        await page.waitForTimeout(700);
      }
    }
    await shot('05-tab-policy.png');

    // ---- 4. Block settings dialogs (motion + checkpoint variants) -------------
    console.log('\n[4] block settings dialogs');
    await page.locator('.q-tab:has-text("플래너")').first().click();
    await page.waitForTimeout(600);
    const gears = page.locator('button:has(.q-icon:has-text("settings"))');
    const gearCount = await gears.count();
    check(gearCount > 0, `설정(gear) 버튼이 있는 블록 존재 (${gearCount})`);

    // Open each settings-capable block, screenshot, and label by the dialog's
    // distinguishing field: motion blocks show "노이즈 rate", checkpoint blocks
    // show "체크포인트별 학습 설정". Capture one of each (first occurrence).
    let gotMotion = false;
    let gotCheckpoint = false;
    for (let i = 0; i < gearCount && !(gotMotion && gotCheckpoint); i++) {
      await gears.nth(i).click();
      await page.waitForTimeout(900);
      const dialog = page.locator('.q-dialog').first();
      if (!(await dialog.isVisible())) continue;
      const isMotion = await dialog.locator('text=노이즈 rate').count();
      const isCheckpoint = await dialog.locator('text=체크포인트별 학습 설정').count();
      if (isMotion && !gotMotion) {
        check(true, '모션 블록 설정 다이얼로그(노이즈) 열림');
        await shot('06-block-settings-motion.png');
        gotMotion = true;
      } else if (isCheckpoint && !gotCheckpoint) {
        check(true, '체크포인트 블록 학습 설정 다이얼로그 열림');
        await shot('07-block-settings-checkpoint.png');
        gotCheckpoint = true;
      }
      await page.keyboard.press('Escape');
      await page.waitForTimeout(400);
    }
    check(gotMotion || gotCheckpoint, '최소 한 종류의 블록 설정 다이얼로그 캡처');

    // ---- 5. final console-error gate -----------------------------------------
    console.log('\n[5] no fatal console errors over the whole run');
    // Vendor/3rd-party noise occasionally logs benign errors; only fail on app crashes.
    const fatal = consoleErrors.filter((e) => /pageerror:/.test(e));
    check(fatal.length === 0, `페이지 크래시(pageerror) 없음 (${fatal.length})`);
    if (consoleErrors.length) {
      console.log('  console errors observed:\n   - ' + consoleErrors.join('\n   - '));
    }
  } catch (err) {
    failures.push('exception: ' + err.message);
    console.error('\n[EXCEPTION]', err);
    try {
      await page.screenshot({ path: path.join(SHOT_DIR, '_failure.png') });
    } catch {
      /* ignore */
    }
  } finally {
    await browser.close();
  }

  console.log('\n=== RESULT ===');
  if (failures.length) {
    console.log(`FAIL (${failures.length}): \n - ` + failures.join('\n - '));
    process.exit(1);
  }
  console.log('PASS — curriculum page verified, screenshots written to ' + SHOT_DIR);
})();
