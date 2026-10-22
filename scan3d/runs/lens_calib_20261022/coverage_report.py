"""Build coverage.md + coverage.json from pose_coverage.py outputs (json/*.json).
usage: coverage_report.py   (run from scan3d/)"""
import json, time
from pathlib import Path

D = Path("runs/lens_calib_20261022")
RECS = [  # (rec, focus label, distance note, round)
    ("hs_20261022_162218", "2.22 dpt", "약 45 cm", "1회차"),
    ("hs_20261022_162754", "1.0 dpt", "약 1 m", "1회차"),
    ("hs_20261022_163008", "3.33 dpt", "약 30 cm", "1회차"),
    ("hs_20261022_163228", "0.37 dpt", "2 m 이상", "1회차"),
    ("hs_20261022_163644", "2.22 dpt", "약 45 cm", "2회차"),
]
# names = where the PHONE is relative to the board (unambiguous); 'up' in pose_coverage = board top edge farther
KO = {"up": "아래에서 올려다보기", "down": "위에서 내려다보기", "left": "오른쪽에서 비스듬히", "right": "왼쪽에서 비스듬히",
      "up-left": "오른쪽 아래에서", "up-right": "왼쪽 아래에서", "down-left": "오른쪽 위에서", "down-right": "왼쪽 위에서"}
SHORT = {"up": "아래에서", "down": "위에서", "left": "오른쪽에서", "right": "왼쪽에서", "up-left": "오른쪽아래에서",
         "up-right": "왼쪽아래에서", "down-left": "오른쪽위에서", "down-right": "왼쪽위에서"}
ORDER = ["up", "down", "left", "right", "up-left", "up-right", "down-left", "down-right"]
CORN = {"top-left": "왼쪽 위", "top-right": "오른쪽 위", "bottom-left": "왼쪽 아래", "bottom-right": "오른쪽 아래"}
EDGE = {"bottom": "화면 맨 아래", "top": "화면 맨 위", "left": "화면 왼쪽 끝", "right": "화면 오른쪽 끝"}


def missing(o):
    """list of (short Korean item, kind) missing vs judge.md requirements."""
    m = []
    P = o["poses"]
    if not any(p["tilt"] < 15 and not p["portrait"] for p in P):
        m.append("정면")
    for d in ORDER:
        has30 = any(p["dir"] == d and 15 <= p["tilt"] < 35 for p in P)
        has45 = any(p["dir"] == d and p["tilt"] >= 35 for p in P)
        if not has30:
            m.append(f"{KO[d]} 30도")
        if not has45:
            mx = max([p["tilt"] for p in P if p["dir"] == d] or [0])
            m.append(f"{KO[d]} 45도" + (f"(최대 {mx:.0f}°)" if mx >= 25 else ""))
    for k, v in CORN.items():
        if o["region_cov"][k] < 0.5:
            m.append(f"{v} 귀퉁이")
    for k, v in EDGE.items():
        if o["region_cov"][k] < 0.6:
            m.append(v)
    if o["n_portrait"] < 3:
        m.append(f"세로 자세 {3 - o['n_portrait']}개 더" if o["n_portrait"] else "세로 자세 3–4개")
    dm = o["dist_m"]
    if dm and dm["min"] > 0.85 * dm["med"]:
        m.append("더 가깝게")
    if dm and dm["max"] < 1.15 * dm["med"]:
        m.append("더 멀게")
    return m


def tilt_table(o):
    rows = ["| 방향 | 0–15° | 15–35° | ≥35° | 최대° |", "|---|---|---|---|---|"]
    for d in ORDER:
        c = o["tilt_poses"][d]
        mx = max([p["tilt"] for p in o["poses"] if p["dir"] == d] or [0])
        rows.append(f"| {KO[d]} | {c['0-15']} | {c['15-35']} | {c['>=35']} | {mx:.0f} |")
    return "\n".join(rows)


def grid3(o):
    c = o["grid3_counts"]
    return "\n".join(["| | 왼 | 가운데 | 오른 |", "|---|---|---|---|"] +
                     [f"| {n} | " + " | ".join(str(x) for x in c[i]) + " |" for i, n in enumerate(["위", "가운데", "아래"])])


def grid32(o):
    return "\n".join("".join("#" if v else "." for v in r) for r in o["grid32"])


res, md_sections, summary_rows, lists = {}, [], [], []
for rec, foc, dist, rnd in RECS:
    f = D / "json" / f"{rec}.json"
    if not f.exists():
        summary_rows.append(f"| {foc} {rnd} | {rec} | (없음) | | | | | |")
        continue
    J = json.loads(f.read_text())
    s, l = J["w<0.05"], J["w<0.1"]
    ms, ml = missing(s), missing(l)
    mlb = {x.split("(")[0] for x in ml}
    shaky = [x for x in ms if x.split("(")[0] not in mlb]          # exists only with 0.05-0.1 rad/s shake
    hard = [x for x in ms if x.split("(")[0] in mlb]
    if s["n_poses"] < 30:
        hard.insert(0, f"정지 자세 {s['n_poses']}/30")
    if s["grid32_cov_corners"] < 0.85:
        hard.append(f"32×18 덮음 {s['grid32_cov_corners'] * 100:.0f}% (<85%)")
    res[rec] = dict(focus=foc, dist_note=dist, round=rnd, missing=hard, shaky_only=shaky,
                    n_poses_005=s["n_poses"], n_poses_01=l["n_poses"], n_still_frames_det12_est=min(s["n_still_det12_est_allframes"], s["n_still"]),
                    cov32_corners=s["grid32_cov_corners"], cov32_hull=s["grid32_cov_hull"], region_cov=s["region_cov"],
                    grid3_counts=s["grid3_counts"], n_portrait=s["n_portrait"], dist_m=s["dist_m"],
                    tilt_poses=s["tilt_poses"], lens_file=s["lens_file"])
    dm = s["dist_m"] or {"min": 0, "med": 0, "max": 0}
    summary_rows.append(f"| {foc} {rnd} | {rec} | {s['n_poses']} ({l['n_poses']}) | {min(s['n_still_det12_est_allframes'], s['n_still'])} | "
                        f"{s['grid32_cov_corners'] * 100:.0f}% ({s['grid32_cov_hull'] * 100:.0f}%) | {s['n_portrait']} | "
                        f"{dm['min']:.2f}–{dm['max']:.2f} (중앙 {dm['med']:.2f}) | {sum(1 for d in ORDER for b in ('15-35', '>=35') if s['tilt_poses'][d][b])}/16 |")
    lists.append(f"- **{foc} {dist} {rnd}** ({rec}): " + ", ".join(hard) +
                 (f". 흔들려서 빠짐(0.05–0.1 rad/s에만 있음): {', '.join(shaky)}" if shaky else ""))
    plist = "\n".join(f"| {p['t0']:.1f} | {p['dur']:.1f} | {p['n']} | {p['tilt']:.0f} | {KO[p['dir']] if p['tilt'] >= 8 else '정면'} | "
                      f"{'세로' if p['portrait'] else '가로'} | {p['dist']:.2f} | {p['cx']:.0f},{p['cy']:.0f} |" for p in s["poses"])
    md_sections.append(f"""### {foc} {dist} {rnd} — {rec}
렌즈 파일 {s['lens_file']}. 정지 프레임(<0.05 rad/s) {s['n_still']}장 중 보드 ≥12 모서리 약 {min(s['n_still_det12_est_allframes'], s['n_still'])}장 (4장마다 1장 검출 ×4 추정).

기울기 분포 (정지 자세 수, 방향 = 보드의 먼 쪽):

{tilt_table(s)}

3×3 칸 모서리 점 수 (0 = 빈 칸):

{grid3(s)}

귀퉁이·가장자리 덮음 (32×18 중 해당 칸 비율): """ + ", ".join(f"{(CORN | EDGE)[k]} {v * 100:.0f}%" for k, v in s["region_cov"].items()) + f"""

32×18 덮음 (# = 모서리 있음):
```
{grid32(s)}
```
<details><summary>정지 자세 목록</summary>

| 시작 s | 길이 s | 모서리 | 기울기° | 방향 | 가로/세로 | 거리 m | 중심 px |
|---|---|---|---|---|---|---|---|
{plist}
</details>
""")

def compress(items):
    """merge per-direction items into short groups."""
    import re
    out, used = [], set()
    for ang in ("30도", "45도"):
        ds = [d for d in ORDER if any(x.startswith(f"{KO[d]} {ang}") for x in items)]
        for d in ds:
            used |= {x for x in items if x.startswith(f"{KO[d]} {ang}")}
        if len(ds) == 8:
            out.append(f"{ang} 8방향 전부")
        elif ds:
            out.append(f"{ang}: " + "·".join(SHORT[d] for d in ds))
    cs = [x.replace(" 귀퉁이", "") for x in items if x.endswith("귀퉁이")]
    used |= {x for x in items if x.endswith("귀퉁이")}
    if cs:
        out.append("귀퉁이: " + "·".join(cs))
    es = [x.replace("화면 ", "") for x in items if x.startswith("화면")]
    used |= {x for x in items if x.startswith("화면")}
    if es:
        out.append("화면 가장자리: " + "·".join(es))
    out += [x for x in items if x not in used and not x.startswith("정지 자세") and not x.startswith("32×18")]
    return out


per_focus = {}
for rec, v in res.items():
    per_focus.setdefault(v["focus"] + " " + v["dist_note"], []).append(v)
combo = []
for k, vs in per_focus.items():
    items = []
    for v in vs:
        for x in v["missing"]:
            x = x.split("(")[0]
            if x not in items:
                items.append(x)
    combo.append(f"- **{k}**: " + "; ".join(compress(items)))
    for v in vs:
        v["reshoot_short"] = compress([x.split("(")[0] for x in v["missing"]])

# combined per focus (two 2.22 rounds count separately for repeatability, but list the union of gaps)
md = f"""# 렌즈 보정 녹화 자세 점검 (2026-10-22)

만든 시각 {time.strftime('%Y-%m-%d %H:%M')}. 스크립트 `runs/lens_calib_20261022/pose_coverage.py` (녹화별 json: `json/`), `coverage_report.py`.
기준: `runs/focus_calib_review/judge.md` (정지 자세 ≥30, 8방향 × 30°/45°, 3×3 전부, 아래 1/6·귀퉁이, 32×18 ≥85 %, 세로 3–4개).

## 결론 먼저
- 다섯 녹화 모두 **45° 자세가 없다**: 정지 자세 중 가장 많이 기울인 것이 26–35°. 가이드의 “45°”가 실제로는 약 30°로 찍혔다 → 다시 찍을 때는 “너무 기울였나” 싶을 만큼 더 기울인다.
- 모두 **화면 귀퉁이(특히 오른쪽 아래·왼쪽 아래·왼쪽 위)와 화면 왼쪽 끝**이 비었다. 32×18 덮음 65–82 % (기준 85 %). 3×3 칸은 모두 채움.
- 세로 자세는 2.22 1회차만 2개(흔들린 것 포함 3개), 나머지는 4–10개로 충분. 정지 자세 수는 2.22 1회차(23), 1.0(29), 0.37(29)이 30 미만.
- 0.37 dpt 녹화는 PnP 거리 1.01–1.15 m (사용자 말 “2 m 이상”과 다름, 렌즈 fx 오차 수 %로는 설명 안 됨 → 줄자 확인 필요)이고 거리 변화도 없다.

## 초점별 다시 찍기 (짧게, 같은 초점 녹화들의 합집합)
{chr(10).join(combo)}

## 녹화별 빠진 자세
{chr(10).join(lists)}

## 요약표
| 초점 | 녹화 | 정지 자세 <0.05 (<0.1) | ≥12 모서리 정지 프레임 | 32×18 덮음 모서리 (보드 면) | 세로 | 거리 m (PnP) | 30°/45° 8방향 칸 |
|---|---|---|---|---|---|---|---|
{chr(10).join(summary_rows)}

## 읽는 법 · 가정
- 정지 = 원시 자이로(type 16, 바이어스 약 0.003 rad/s 안 뺌) 크기 < 0.05 rad/s, 프레임 t_mid_ns에 시간 보간. 0.1 rad/s 결과는 괄호/“흔들려서 빠짐”으로 따로.
- 정지 자세 = 연속 정지 구간(틈 < 0.15 s, 길이 ≥ 0.3 s), 그 안에서 보드 위치가 150 px 넘게·기울기 8° 넘게 바뀌면 나눔. 대표 프레임 한 장으로 기울기·방향·거리.
- 자세 추정: solvePnP (IPPE → 반복), 렌즈 파일은 초점이 가장 가까운 것 (f0.37 / f2.22만 있음 → 1.0 dpt는 f0.37, 3.33은 f2.22). 거리는 렌즈 fx 오차(수 %)만큼 틀릴 수 있음 (가정).
- 기울기 = 보드 법선과 카메라 축 사이 각. **방향 이름 = 보드에 대해 폰이 있는 쪽** (가이드의 “위/아래로 기울임”은 사람마다 뜻이 갈려서 이렇게 씀). “아래에서 올려다보기” = 폰이 보드보다 낮음 = 영상에서 보드 위쪽이 작게(멀게) 보임. “오른쪽에서 비스듬히” = 폰이 보드 오른쪽 = 보드 왼쪽이 작게 보임. 대각도 같은 식. 세로 자세에서도 영상 기준. (프레임 2956·3826에서 눈으로 확인: 보드 아래쪽/오른쪽이 작게 보이는 프레임이 각각 “위에서 내려다보기”/“왼쪽에서 비스듬히”로 분류됨.)
- 1회차 2.22 녹화의 순서를 보면 가이드 첫 기울기(“위로 30°”)에서 실제로는 “위에서 내려다보기”가 찍힘 → 가이드의 위/아래 뜻이 반대로 읽혔을 수 있음.
- 30° 칸 = 15–35°, 45° 칸 = ≥35°. 귀퉁이 = 32×18 중 귀퉁이 4×3칸(가로 1/8 × 세로 1/6)의 50 % 미만이면 빠짐, 가장자리 띠(3줄 또는 4열)는 60 % 미만이면 빠짐.
- 세로 자세 = 보드 가로축이 영상에서 45–135° 돌아감. 더 가깝게/멀게 = 최소 거리 > 중앙값의 0.85배, 최대 < 1.15배일 때 빠짐.
- 32×18 덮음은 “모서리 점이 들어간 칸” 기준(judge 기준). 괄호 안은 보드 면이 덮은 칸(참고).

## 녹화별 자세한 결과
{chr(10).join(md_sections)}
"""
(D / "coverage.md").write_text(md)
(D / "coverage.json").write_text(json.dumps(dict(made=time.strftime("%Y-%m-%d %H:%M:%S"), criteria="runs/focus_calib_review/judge.md",
                                                 recordings=res), indent=1, ensure_ascii=False))
print("\n".join(lists)); print(); print("\n".join(combo))
