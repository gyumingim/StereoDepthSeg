"""python pf.py TAG NNC IN GOLD  -> remote Exynos 2400 profile (Duration 5 s) in my own tab, log saved prof/TAG.txt"""
import sys, os, time, re
from playwright.sync_api import sync_playwright
tag, nnc, fin, fg = sys.argv[1:5]
os.makedirs('prof', exist_ok=True)
p = sync_playwright().start(); b = p.chromium.connect_over_cdp("http://127.0.0.1:9333")
ctx = b.contexts[0]
pgs = [x for x in ctx.pages if 'NNC_PROFILER' in x.url]
pg = pgs[-1] if len(pgs) > 1 else ctx.new_page()
pg.goto("https://soc-developer.semiconductor.samsung.com/global/development/ai-studio/project/?tab=NNC_PROFILER", timeout=60000)
pg.wait_for_timeout(5000)
sel = pg.query_selector("select")
opts = [o.inner_text() for o in sel.query_selector_all("option")]
v = [o.get_attribute("value") for o in sel.query_selector_all("option") if '2400' in o.inner_text()][0]
sel.select_option(v)
ins = pg.query_selector_all("input[type=file]")
ins[0].set_input_files(nnc); pg.wait_for_timeout(2000)
ins[1].set_input_files(fin); pg.wait_for_timeout(2000)
ins[2].set_input_files(fg); pg.wait_for_timeout(3000)
t0 = time.time()
pg.get_by_role("button", name="Run").click()
st = ''
for i in range(120):
    pg.wait_for_timeout(5000)
    st = pg.inner_text("body")
    if re.search(r'TEST SUCCESS|TEST FAIL|FAILED|Average execution', st): break
pg.wait_for_timeout(3000); st = pg.inner_text("body")
m = re.findall(r'(TEST \w+|EXECUTION FAILED|Average[^\n]*)', st)
print(tag, time.strftime('%H:%M:%S'), round(time.time() - t0), m[:6])
try:
    with pg.expect_download(timeout=30000) as d:
        pg.get_by_role("button", name="Log Download").click()
    d.value.save_as(f'prof/{tag}.txt'); print('LOG', f'prof/{tag}.txt')
except Exception as e:
    print('NO_LOG', str(e)[:100])
pg.screenshot(path=f'prof/{tag}.png')
