"""Morning report 2026-10-20 → scan3d/docs/report_20261020.html (reads runs/*.log / *.json produced overnight)."""
import ast
import glob
import json
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
RUNS = HERE.parent / "runs"
OUT = HERE.parent / "docs" / "report_20261020.html"
NAMES = {"hs_20261019_115620": "2번", "hs_20261019_131326": "4번", "hs_20261019_145816": "6번", "hs_20261019_154320": "9번", "hs_20261019_161035": "10번"}
STAGES = [("보정 없음", "none"), ("바이어스만", "bias"), ("물리 보정", "calib"), ("물리 보정 + CNN", "calib_cnn")]


def parse_train(path):
    out = {}
    for line in Path(path).read_text(errors="ignore").splitlines():
        for label, key in STAGES:
            if line.startswith(label + ":") or (key == "calib" and line.startswith("물리 보정 (")):
                if key == "calib" and line.startswith("물리 보정 +"):
                    continue
                try:
                    d = ast.literal_eval(line.split(":", 1)[1].strip())
                except Exception:
                    continue
                out[key] = d["val"]
                if key == "calib":
                    out["lever"] = d.get("r")
    return out


def train_table():
    rows = {}
    for gt in ("old", "rs"):
        for f in sorted(glob.glob(str(RUNS / f"overnight_train_{gt}_*.log"))):
            n = Path(f).stem.split(f"overnight_train_{gt}_")[1]
            rows.setdefault(n, {})[gt] = parse_train(f)
    return rows


def fmt(v, d=2):
    return "-" if v is None else f"{v:.{d}f}"


def bars(items, unit, maxv=None, w=520):
    """items: list of (label, value, color var). Simple horizontal bars in inline SVG."""
    items = [i for i in items if i[1] is not None]
    if not items:
        return ""
    mv = maxv or max(v for _, v, _ in items) * 1.1
    h = 26 * len(items) + 10
    s = [f'<svg viewBox="0 0 {w} {h}" role="img">']
    for k, (lab, v, col) in enumerate(items):
        y = 6 + 26 * k
        bw = (w - 230) * v / mv
        s.append(f'<text x="0" y="{y + 15}">{lab}</text><rect x="170" y="{y + 2}" width="{bw:.1f}" height="18" rx="4" fill="var({col})"/>'
                 f'<text x="{176 + bw:.1f}" y="{y + 15}">{v:.2f} {unit}</text>')
    s.append("</svg>")
    return "".join(s)


def vio():
    res = {}
    for f in glob.glob(str(RUNS / "vio" / "*.json")):
        try:
            res[Path(f).stem] = json.load(open(f))
        except Exception:
            pass
    return res


def main():
    T = train_table()
    avg = {}
    for gt in ("old", "rs"):
        for _, key in STAGES:
            vals = [T[n][gt][key]["pos_rms_cm"] for n in T if gt in T[n] and key in T[n][gt]]
            avg[(gt, key)] = (sum(vals) / len(vals), len(vals)) if vals else (None, 0)
    # CNN vs physical calibration on the SAME held-out recordings (some folds have no CNN result)
    common = [n for n in T if "rs" in T[n] and "calib_cnn" in T[n]["rs"] and "calib" in T[n]["rs"]]
    cc = sum(T[n]["rs"]["calib"]["pos_rms_cm"] for n in common) / max(1, len(common))
    cn = sum(T[n]["rs"]["calib_cnn"]["pos_rms_cm"] for n in common) / max(1, len(common))
    rows = []
    for n in NAMES:
        if n not in T:
            continue
        for gt in ("old", "rs"):
            r = T[n].get(gt, {})
            cells = "".join(f"<td>{fmt(r.get(k, {}).get('pos_rms_cm'))} / {fmt(r.get(k, {}).get('rot_end_deg'), 2)}°</td>" for _, k in STAGES)
            rows.append(f"<tr><td>{NAMES[n]}</td><td>{'기존' if gt == 'old' else '롤링셔터 폄'}</td>{cells}</tr>")
    levers = [T[n][gt].get("lever") for n in T for gt in T[n] if T[n][gt].get("lever")]
    V = vio()

    def vrow(tag):
        out = []
        for n in ("hs_20261019_131326", "hs_20261019_145816"):
            r = V.get(f"{n}_{tag}")
            if r and "ate_cm" in r:
                out.append(f"{NAMES[n]}: ATE {r['ate_cm']} cm · RPE1s {r.get('rpe1s_cm')} cm · 회전 {r.get('rot_med_deg')}° · 추적 {r['tracked_frac'] * 100:.0f}% · 스케일 {r['scale']}"
                           + (f" · 끊김 최대오차 {r['dropout_max_err_cm']} cm" if r.get("dropout_max_err_cm") else ""))
            elif r:
                out.append(f"{NAMES[n]}: {r.get('error', '실패')}")
        return "<br>".join(out) or "-"
    vio_tags = [("fps30", "30 fps"), ("fps60", "60 fps"), ("fps120", "120 fps"), ("size480", "60 fps · 480×270"),
                ("noisemeas", "60 fps · 잡음 실측값 그대로 (발산)"), ("noisemeas_x5", "60 fps · 잡음 실측 ×5"),
                ("noisemeas_x10", "60 fps · 잡음 실측 ×10"), ("fps120_clones22", "120 fps · 기억 사진 22장"),
                ("fps120_clones33", "120 fps · 기억 사진 33장"), ("calib", "60 fps · 물리 보정 IMU"),
                ("calib_noisemeas", "60 fps · 물리 보정 + 잡음 실측"), ("drop_raw", "카메라 끊김 2/5/10 s · 생 IMU"), ("drop_raw_x10", "카메라 끊김 · 생 IMU + 잡음 실측 ×10"),
                ("drop_calib", "카메라 끊김 2/5/10 s · 보정 IMU + 실측 잡음")]
    vio_rows = "".join(f"<tr><td>{lab}</td><td>{vrow(tag)}</td></tr>" for tag, lab in vio_tags)
    timing = []
    for f in sorted(glob.glob(str(RUNS / "vio" / "timing_*.json"))):
        r = json.load(open(f))
        timing.append(f"<tr><td>{r['fps']:.0f} fps · {r['size']} · 특징점 {r['npts']}</td><td>{r.get('ms_med')} ms</td><td>{r.get('ms_p90')} ms</td>"
                      f"<td>{(r.get('ms_med') or 0) * PHONE_RATIO:.1f} ms</td><td>{1000 / max(1e-6, (r.get('ms_med') or 1) * PHONE_RATIO * 1.4):.0f}</td></tr>")
    gte = {}
    for f in glob.glob(str(RUNS / "gt_experiment_hs_*.json")):
        gte[Path(f).stem.replace("gt_experiment_", "")] = json.load(open(f))
    gte_rows = "".join(
        f"<tr><td>{NAMES.get(n, n[-6:])}</td><td>{d['base']['gyro_deg']}° / {d['base']['gyro_fast_deg']}°</td>"
        f"<td><b>{d['rs']['gyro_deg']}° / {d['rs']['gyro_fast_deg']}°</b></td><td>{d['rs_neg']['gyro_deg']}°</td>"
        f"<td>{d['base']['frames']} → {d['rs']['frames']}</td></tr>" for n, d in sorted(gte.items()) if "rs" in d)
    sc = []
    p = RUNS / "scale_check_imu_gt.json"
    if p.exists():
        sc = json.load(open(p))
    sc_txt = ", ".join(f"{NAMES.get(r['rec'], r['rec'])} {r['s_best']}" for r in sc)
    a = lambda gt, k: avg[(gt, k)][0]
    html = TEMPLATE.format(
        n_folds=len([n for n in T if 'rs' in T[n]]),
        none_old=fmt(a("old", "none")), none_rs=fmt(a("rs", "none")), bias_old=fmt(a("old", "bias")), bias_rs=fmt(a("rs", "bias")),
        cal_old=fmt(a("old", "calib")), cal_rs=fmt(a("rs", "calib")), cnn_old=fmt(a("old", "calib_cnn")), cnn_rs=fmt(a("rs", "calib_cnn")),
        n_common=len(common), cc=fmt(cc), cn=fmt(cn), cnn_delta=f"{(cn / cc - 1) * 100:+.1f}" if common else "-",
        cal_gain=f"{(1 - a('rs', 'calib') / a('rs', 'none')) * 100:.0f}" if a("rs", "calib") else "-",
        cal_vs_bias=f"{(1 - a('rs', 'calib') / a('rs', 'bias')) * 100:.0f}" if a("rs", "calib") else "-",
        rs_gain=f"{(1 - a('rs', 'calib') / a('old', 'calib')) * 100:.0f}" if a("rs", "calib") else "-",
        bars_main=bars([("보정 없음", a("rs", "none"), "--mut"), ("바이어스만", a("rs", "bias"), "--d"),
                        ("물리 보정", a("rs", "calib"), "--c"), ("물리 보정 + CNN*", a("rs", "calib_cnn"), "--p")], "cm"),
        train_rows="".join(rows), lever=", ".join("(" + ", ".join(f"{v * 100:.1f}" for v in l) + ")" for l in levers[:4]),
        vio_rows=vio_rows, timing_rows="".join(timing) or "<tr><td colspan=5>측정 중/실패</td></tr>",
        gte_rows=gte_rows, scale=sc_txt or "-")
    OUT.write_text(html, encoding="utf-8")
    print("→", OUT)


PHONE_RATIO = 1.5   # phone ms / PC ms, ESTIMATE: single-core benchmark gap ~1.3x (7800X3D vs S24) plus slower memory; verify on the phone

TEMPLATE = """<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>밤사이 결과 10-20</title><style>
:root{{--bg:#f7f7f5;--fg:#1c1f24;--mut:#6b7078;--card:#fff;--line:#e2e4e8;--a:#2563eb;--b:#dc2626;--c:#16a34a;--d:#d97706;--p:#9333ea;--chip:#eef2ff}}
@media (prefers-color-scheme:dark){{:root:not([data-theme="light"]){{--bg:#121417;--fg:#e8eaed;--mut:#9aa0a6;--card:#1b1e23;--line:#2c3037;--a:#60a5fa;--b:#f87171;--c:#4ade80;--d:#fbbf24;--p:#c084fc;--chip:#1e293b}}}}
body{{margin:0;background:var(--bg);color:var(--fg);font:17px/1.7 system-ui,"Malgun Gothic",sans-serif}}main{{max-width:980px;margin:0 auto;padding:20px 16px 80px}}
h1{{font-size:27px;margin:0 0 4px}}h2{{font-size:21px;margin:36px 0 8px;padding-top:8px;border-top:1px solid var(--line)}}
.card{{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:16px;margin:12px 0}}.key{{border-left:5px solid var(--a);padding:8px 14px;background:var(--chip);border-radius:0 10px 10px 0;margin:10px 0}}
table{{border-collapse:collapse;width:100%;font-size:14.5px}}th,td{{border-bottom:1px solid var(--line);padding:6px 7px;text-align:left;vertical-align:top}}th{{color:var(--mut);font-size:13.5px}}
.scroll{{overflow-x:auto}}svg{{width:100%;max-width:620px;height:auto;display:block}}svg text{{fill:var(--fg);font:13px system-ui,sans-serif}}.small{{font-size:14px;color:var(--mut)}}
.good{{color:var(--c);font-weight:700}}.bad{{color:var(--b);font-weight:700}}
</style></head><body><main>
<h1>밤사이 결과 (2026-10-20 아침)</h1>
<p class="small">녹화 11개 (약 43분, 45 cm 고정 초점 8개) · 평가는 학습에 안 쓴 녹화로만 · 숫자는 3초 창 위치 오차 (보드 정답 대비, 중앙값)</p>

<div class="key"><b>한 줄 결론</b> — <b>물리 보정(숫자 27개)이 IMU 오차를 약 {cal_gain}% 줄였고, 처음 보는 녹화에서도 그대로 통했습니다.</b>
CNN은 데이터가 10배가 되어도 거의 더해 주지 못했습니다 → <b>CNN은 접고 물리 보정을 확정</b>. 정답의 롤링셔터를 펴니 보정 결과가 {rs_gain}% 더 좋아졌습니다 (정답 품질이 실제로 영향 있음).</div>

<h2>1. IMU 보정 — 처음 보는 녹화에서 (롤링셔터 편 정답, 평균 {n_folds}개)</h2>
<div class="card">{bars_main}
<p class="small">* CNN 막대는 CNN 결과가 있는 녹화만의 평균이라 위 막대들과 녹화 구성이 조금 다름.</p><p>보정 없음 {none_rs} cm → 바이어스만 {bias_rs} cm → <span class="good">물리 보정 {cal_rs} cm</span> (바이어스만보다 {cal_vs_bias}% 좋음) → CNN 을 더하면: 같은 평가 녹화 {n_common}개에서 물리 보정 {cc} cm → 물리+CNN {cn} cm ({cnn_delta}%, 사실상 0).
판단 기준이던 "바이어스만보다 30% 이상"은 <b>물리 보정만으로 거의 도달</b>, CNN 추가분은 0에 가까움.</p>
<div class="scroll"><table><tr><th>평가 녹화</th><th>정답</th><th>보정 없음</th><th>바이어스만</th><th>물리 보정</th><th>물리+CNN</th></tr>{train_rows}</table></div>
<p class="small">칸 = 위치 오차 cm / 3초 끝 회전 오차. 기존 정답 평균: 없음 {none_old} · 바이어스 {bias_old} · 물리 {cal_old} · +CNN {cnn_old} cm.
물리 보정이 찾은 카메라-IMU 거리(레버암, IMU 축 cm): {lever} — 학습 녹화를 바꿔도 거의 같은 값 = 그럴듯한 물리값.</p>
</div>

<h2>2. 정답(GT) 품질</h2>
<div class="card"><b>롤링셔터 펴기</b> (자이로로 줄마다 찍힌 순간을 맞춤) — 독립 잣대(자이로 0.2초 회전 일치)로 검증:
<div class="scroll"><table><tr><th>녹화</th><th>기존 (전체/빠른 구간)</th><th>폄</th><th>반대 방향 (확인용)</th><th>쓸 수 있는 사진</th></tr>{gte_rows}</table></div>
<p>모든 녹화에서 좋아지고, 일부러 반대로 하면 나빠짐 → 계산 방향까지 맞음. 모서리 추가 정밀화는 이득 없어 안 씀.</p>
<p><b>렌즈 값</b>: 고정 초점 녹화 8개로 다시 구해 봤지만 시험 사진에서 기존 값보다 나빠서 유지. 다만 두 값의 초점거리가 2% 다르고(1267 vs 1291) 평평한 보드로는 구분이 안 됨 →
<b>가속도계로 정답 크기 확인</b>: 최적 배율 {scale} (모두 1~3% 안, 곡선이 평평) → 정답 크기는 약 ±2% 안에서 맞음.</p></div>

<h2>3. VIO 시험대 (OpenVINS, 보드 정답 대비, 60초씩)</h2>
<div class="card"><div class="scroll"><table><tr><th>설정</th><th>결과</th></tr>{vio_rows}</table></div>
<p><b>읽는 법</b> — ① 30·60 fps, 480×270 은 정상 (1~2.6 cm). ② <b>120 fps 는 발산</b>: OpenVINS 가 기억하는 과거 사진 11 장이 120 fps 에선 0.09 초뿐이라
시차가 모자람 — 22·33 장으로 늘리면 11 m → 18 cm → 10 cm 로 나아지지만 아직 부족 → <b>추적은 120 fps, 갱신은 더 긴 간격</b>으로 나누는 수정이 필요 (오늘 할 일).
③ <b>IMU 잡음 실측값을 그대로 넣으면 발산</b> (기존 설정보다 자이로 8 배·가속도 20 배 작음 → IMU 를 너무 믿음). <b>실측 ×10 이 기존 설정보다 좋음</b>
(전체 오차 0.90 vs 0.99 cm, 회전 0.41° vs 0.57°) — open_vins#65 (잡음 설정이 결과를 좌우) 와 같은 결론. ④ <b>카메라 끊김 숫자 주의</b>: OpenVINS 는 사진이 들어올 때만 위치를 내놓아서, 표의 "끊김 최대오차"는 끊긴 동안의 IMU 표류가 아니라
<b>카메라가 돌아온 직후</b>의 오차입니다 (0.6~2.6 m). 즉 끊긴 뒤 OpenVINS 가 옛 특징점과 다시 맞추다 크게 튀는 <b>복구 문제</b>이고, 잡음 설정(×10)으로도 거의 안 바뀜
(2 초 끊김 78 → 62 cm). 끊긴 동안의 순수 IMU 표류는 위 1번의 3 초 창 오차 (물리 보정 0.99 cm) 가 더 가까운 숫자. → 오늘 할 일: 끊김 뒤 재초기화·특징점 리셋.</p>
<p class="small">ATE = 전체 위치 오차, RPE1s = 1초 동안 흘러간 오차, 스케일 = VIO 크기/정답 크기. 물리 보정은 그 녹화를 빼고 학습한 값만 사용(공정).</p>
<b>속도 (PC 실측 → 폰 추정)</b>
<div class="scroll"><table><tr><th>설정</th><th>PC 중앙</th><th>PC 상위10%</th><th>폰 추정 (중앙)</th><th>폰 지속 가능 fps (발열 ×1.4)</th></tr>{timing_rows}</table></div>
<p class="small">폰 추정 = PC 시간 × 1.5 (추정: 7800X3D 대 S24 단일 코어 성능 차 약 1.3 배 + 느린 메모리), 지속 가능 fps 는 발열로 1.4 배 더 느려진다고 가정.
예전 폰 실측(640×480 단안 13.1 ms)은 설정이 달라 비율 계산에 못 씀 — <b>오늘 폰으로 꼭 확인할 항목</b>. 시간 측정용 실행은 30 초짜리라 정확도 열은 60 초 표와 다름.</p></div>

<h2>4. 밤사이 사고와 고친 것</h2>
<div class="card"><ul>
<li>정답 만드는 스크립트가 지워져 있었음 (Windows 쪽 셸로 WSL 파일을 고친 것이 원인) → 복구, 규칙으로 저장.</li>
<li>회전 곡선이 180° 넘게 돌면 깨지는 버그 (6번 녹화 6°) → 1초 조각으로 고쳐 0.12°.</li>
<li>학습 8개를 동시에 돌렸더니 예상보다 훨씬 느려 VIO 비교를 줄여서 실행 (녹화 2개 × 60초).</li>
<li>WSL 쪽 git 은 GitHub 로그인이 없어 그동안 push 가 안 됐음 → Windows git 으로 push (전부 올라감).</li></ul></div>

<h2>5. 오늘 할 것 / 정할 것</h2>
<div class="card"><ol>
<li><b>OpenVINS 설정 확정</b>: IMU 잡음 = 실측 ×10 (두 녹화 모두 좋아짐), 물리 보정값(축척·축·바이어스·레버암) + 시간차 −2 ms. 폰 설정 파일로 만들어 둠.</li>
<li><b>120 fps 를 쓰려면 OpenVINS 수정</b>: 추적은 매 장, 갱신(복제 상태)은 더 긴 간격 — 지금 그대로는 발산.</li>
<li><b>카메라 끊김 뒤 복구 개선</b>: 돌아온 직후 0.6~2.6 m 튐 → 끊김 감지 시 특징점 리셋·상태 유지 재시작 (ov_bridge 의 reinit-state 활용).</li>
<li><b>폰에서 OpenVINS 속도 실측</b> (60 fps, 해상도·특징점 수별) — 폰 연결만 해 주시면 제가 돌림.</li>
<li>CNN 은 접기 (데이터 10 배에도 +1.6 %). (선택) 넓은 실제 공간 VIO 녹화.</li></ol></div>
</main></body></html>"""

if __name__ == "__main__":
    main()
