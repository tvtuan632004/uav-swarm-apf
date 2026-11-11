# uav-swarm-apf

Điều khiển đội hình và tránh va chạm cho bầy UAV (mô hình tích phân kép), dùng
trường thế nhân tạo (APF) cải tiến với bốn thành phần:

| Mã | Thành phần | Ý tưởng |
|---|---|---|
| T1 | Thế đẩy chuẩn hóa theo đích | Lực đẩy bằng 0 khi đích nằm sát vật cản (hết lỗi GNRON) |
| T2 | Thế Morse + rào chắn | Cân bằng tại khoảng cách đội hình d\*, không cặp nào gần hơn d_min |
| T3 | Lực con quay + reset khi kẹt | Lực vuông góc vận tốc để thoát cực tiểu địa phương |
| T4 | Giảm chấn Hessian chiếu PSD | Giảm dao động mà không bơm năng lượng |

Phương pháp được so với 5 baseline (B1 APF Khatib, B2 Olfati-Saber, B3 Hessian đầy đủ,
B4 HOCBF-QP, B5 ORCA) trên 5 kịch bản S1–S5. **`paper/main.tex` là nguồn chuẩn duy
nhất cho mọi công thức.** Code luôn trích số phương trình tương ứng.

---

## Mục lục
1. [Trạng thái](#1-trạng-thái)
2. [Yêu cầu](#2-yêu-cầu)
3. [Cài đặt nhanh trên Windows](#3-cài-đặt-nhanh-trên-windows)
4. [Cài đặt thủ công (Windows / Ubuntu)](#4-cài-đặt-thủ-công-windows--ubuntu)
5. [Lệnh dùng hằng ngày](#5-lệnh-dùng-hằng-ngày)
6. [Cấu trúc thư mục](#6-cấu-trúc-thư-mục)
7. [Dùng thư viện](#7-dùng-thư-viện)
8. [Thư mục kết quả](#8-thư-mục-kết-quả-uavapf_results)
9. [Quy trình Claude + Codex](#9-quy-trình-claude--codex)
10. [Quy ước code](#10-quy-ước-code)
11. [Git](#11-git)
12. [Xử lý sự cố](#12-xử-lý-sự-cố)

---

## 1. Trạng thái

Bảng nhiệm vụ, vai trò và lịch đầy đủ: **`docs/plan.md`**. Tóm tắt (11/11/2026):

| Mã | Nhiệm vụ | Trạng thái |
|---|---|---|
| T01–T04 | Repo, mô phỏng RK4, hình học vật cản | Xong, chờ Codex review |
| T07–T11 | Bộ điều khiển APF T1–T4 + reset (`uavapf.control`) | Xong, đã review độc lập |
| T05, T06, T11b, T11c, T12 | YAML + hình, baseline B1, va chạm là kết quả, tăng tốc, 7 chỉ số | Xong trong đêm, chờ Codex review |
| T13 | Sinh S1–S5 theo seed | Bản nháp, chờ chốt tham số |
| T13b | Tâm đội hình di chuyển | Code và test đã có; chờ pilot và review |
| T14–T20 | B2–B5, Monte Carlo, tinh chỉnh, PX4 | Chưa làm |

Kiểm tra hiện tại: **312 test xanh** (Windows, `.venv-codex`); lint và format sạch.
Kết quả pilot APF so với B1:
`docs/pilot-2026-10-11.md`.

---

## 2. Yêu cầu

| Thành phần | Phiên bản | Ghi chú |
|---|---|---|
| Hệ điều hành | Windows 10/11 hoặc Ubuntu 22.04+ | CI chạy trên cả hai |
| [uv](https://docs.astral.sh/uv/) | ≥ 0.5 | Quản lý Python và thư viện; script cài tự cài uv |
| Python | 3.12 | uv tự tải về, **không** dùng Python của conda |
| Git | bất kỳ | Để quản lý phiên bản và đẩy lên GitHub |

Thư viện chạy (khóa phiên bản chính xác trong `uv.lock`): `numpy`, `scipy`, `pyyaml`,
`matplotlib`. Thư viện phát triển: `pytest`, `ruff`.

> Máy đang có Anaconda/Miniconda thì không sao. uv tạo môi trường riêng trong `.venv/`
> và không đụng tới conda. Chỉ cần **không** kích hoạt môi trường conda khi chạy
> `uv run ...`. Nếu lỡ kích hoạt, gõ `conda deactivate` trước.

---

## 3. Cài đặt nhanh trên Windows

**Cách 1: bấm đúp** vào `scripts\setup.cmd`.

**Cách 2: dùng PowerShell** ở thư mục gốc repo (`C:\Setup\UAV`):

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\setup.ps1
```

Script làm lần lượt các việc sau và dừng ngay khi có bước lỗi:

1. Cài `uv` nếu máy chưa có (dùng `winget`, nếu không có thì dùng `pip install --user uv`).
2. `uv python install 3.12`: tải Python 3.12 riêng cho uv.
3. `uv sync --locked`: tạo `.venv\` và cài đúng các phiên bản trong `uv.lock`.
4. `ruff check`, `ruff format --check`, `pytest`, rồi chạy `scripts\demo.py`.

Mọi thứ in ra màn hình cũng được ghi vào `setup-report.txt` (đã có trong `.gitignore`).
Dòng cuối là `ALL DONE - environment ready.` thì môi trường đã sẵn sàng.

Nếu chỉ muốn cài mà không chạy kiểm tra: thêm `-SkipChecks`. Muốn xóa `.venv` và dựng lại từ đầu: thêm `-Clean`.

Sau bước `uv sync`, script kiểm tra xem Python trong `.venv` có khởi động được không. Nếu bản Python gốc đã bị xóa hoặc di chuyển (ví dụ `.venv` được tạo từ sandbox của Codex), script tự cài lại Python 3.12 và dựng lại `.venv` một lần.

---

## 4. Cài đặt thủ công (Windows / Ubuntu)

```powershell
# Windows (PowerShell)
winget install --id astral-sh.uv -e      # hoặc: pip install --user uv
# mở cửa sổ PowerShell mới để PATH cập nhật
cd C:\Setup\UAV
uv sync                                   # tạo .venv và cài thư viện
uv run pytest -q
```

```bash
# Ubuntu
curl -LsSf https://astral.sh/uv/install.sh | sh
cd ~/uav-swarm-apf
uv sync
uv run pytest -q
```

`uv sync` đọc `.python-version` (3.12), tự tải Python nếu thiếu, rồi cài theo `uv.lock`.
Chạy lại lệnh này bất cứ lúc nào `pyproject.toml` hoặc `uv.lock` thay đổi.

**Dùng với VS Code:** `Ctrl+Shift+P` → *Python: Select Interpreter* →
chọn `.venv\Scripts\python.exe`. Sau đó Testing panel sẽ tự nhận pytest.

---

## 5. Lệnh dùng hằng ngày

| Việc | Lệnh |
|---|---|
| Cập nhật môi trường | `uv sync` |
| Chạy toàn bộ test | `uv run pytest -q` |
| Chạy test một nhiệm vụ | `uv run pytest -q tests/test_t03_sim.py` |
| Chạy một test theo tên | `uv run pytest -q -k energy` |
| Kiểm tra lint | `uv run ruff check .` |
| Tự sửa lỗi lint đơn giản | `uv run ruff check . --fix` |
| Kiểm tra định dạng | `uv run ruff format --check .` |
| Tự định dạng | `uv run ruff format .` |
| Thêm thư viện | `uv add <tên>` (thư viện dev: `uv add --dev <tên>`) |
| Chạy demo | `uv run python scripts/demo.py` |
| Chạy một kịch bản YAML (npz + png + gif) | `uv run python scripts/run_scenario.py configs/scenarios/demo_sphere.yaml` |
| Pilot APF so với B1 trên S1–S5 | `uv run python scripts/pilot.py --seeds 3 --n 5 10` |
| Đo tốc độ controller | `uv run python scripts/bench_controller.py` |

**Không dùng `pip install`** trong repo này. Mọi thư viện phải đi qua `uv add`, để
`uv.lock` và CI luôn khớp với máy bạn.

---

## 6. Cấu trúc thư mục

```
uav-swarm-apf/
├── AGENTS.md              luật chung cho Codex và Claude (đọc đầu tiên)
├── CLAUDE.md              @AGENTS.md + ghi chú riêng cho Claude
├── README.md              file này
├── pyproject.toml         metadata, thư viện, cấu hình ruff/pytest
├── uv.lock                khóa phiên bản (commit file này)
├── .python-version        3.12
├── .github/workflows/ci.yml   CI: ruff + pytest trên Windows và Ubuntu
├── configs/               YAML tham số, kịch bản, dải seed (từ T05)
├── src/uavapf/
│   ├── paths.py           thư mục kết quả (biến UAVAPF_RESULTS)
│   ├── sim/               T03: mô phỏng tích phân kép, RK4, bão hòa, Trajectory
│   ├── geometry/          T04: vật cản cầu/trụ, khoảng cách ρ, pháp tuyến n̂
│   ├── forces/            T06–T10: các thế và lực
│   ├── control/           APF T1–T4, giảm chấn Hessian PSD, reset khi kẹt
│   ├── baselines/         T14–T15: B1–B5
│   ├── scenarios/         T13: S1–S5
│   └── metrics/           T12: 7 chỉ số đánh giá
├── tests/                 Claude viết; người cài đặt không sửa
├── scripts/
│   ├── setup.ps1, setup.cmd   cài đặt + kiểm tra trên Windows
│   └── demo.py            ví dụ nhỏ dùng T03 + T04
├── docs/
│   ├── specs/             đặc tả từng nhiệm vụ (T01, T03, T04, _template)
│   ├── handoff.md         nhật ký bàn giao, mới nhất ở trên
│   └── decisions.md       quyết định thiết kế D000, D001, ...
├── paper/                 main.tex, references.bib (đồng bộ Overleaf)
└── results/               KHÔNG commit (trong .gitignore)
```

---

## 7. Dùng thư viện

### 7.1 Mô phỏng (T03, `uavapf.sim`)

Mô hình cho mỗi UAV i, với bão hòa theo chuẩn (M1–M2 trong `docs/specs/T03-sim.md`):

    ṗ_i = v_i,    v̇_i = sat(u_i(t, p, v), a_max),    sat(x, L) = x · min(1, L/‖x‖)

```python
import numpy as np
from uavapf.sim import SimConfig, simulate

goals = np.array([[10.0, 0, 5], [10.0, 2, 5], [10.0, -2, 5]])  # m


def controller(t, p, v):  # p, v: (N, 3) -> u: (N, 3) m/s^2
    return 0.6 * (goals - p) - 1.4 * v


cfg = SimConfig(dt=0.01, t_final=30.0, a_max=3.0, v_max=5.0, log_every=10)
traj = simulate(controller, p0=np.zeros((3, 3)), v0=np.zeros((3, 3)), config=cfg)

traj.t  # (K,)        s
traj.p  # (K, N, 3)   m
traj.v  # (K, N, 3)   m/s
traj.u  # (K, N, 3)   m/s^2, lệnh đã bão hòa tại đúng trạng thái đã log
```

Các điểm cần biết:
- Bộ điều khiển nhận **toàn bộ** trạng thái bầy và được gọi ở cả 4 bước con của RK4.
  Đây là điều khiển liên tục, không giữ lệnh qua cả bước. Bão hòa `a_max` nằm bên
  trong phương trình, không chỉ áp lên giá trị được log.
- `v_max` là phép chiếu sau mỗi bước. Phép chiếu này phá bảo toàn năng lượng, nên tắt
  (`None`) khi kiểm tra năng lượng.
- Số bước là `ceil(t_final / dt)`. Thời điểm log là `k·dt` (không cộng dồn sai số).
  Trạng thái cuối luôn được log.
- `stop=lambda t, p, v: ...` trả `True` thì dừng sớm, ví dụ khi tất cả đã tới đích
  hoặc khi có va chạm.
- Chi phí: 4 lần gọi controller mỗi bước, cộng 1 lần cho trạng thái cuối. 50 UAV ×
  6000 bước với controller đơn giản chạy khoảng 0,1 s.

Lưu và đọc lại (thư mục cha được tạo tự động; metadata đi kèm file):

```python
from uavapf.paths import results_root
from uavapf.sim import Trajectory

path = traj.save(results_root() / "S1" / "seed_10000.npz", seed=10000, scenario="S1")
traj2 = Trajectory.load(path)
```

### 7.2 Vật cản (T04, `uavapf.geometry`)

```python
import math
from uavapf.geometry import Cylinder, ObstacleSet, Sphere

obstacles = ObstacleSet(
    [
        Cylinder(center_xy=(20, 6), radius=2.0, z_min=-math.inf, z_max=30.0),  # tòa nhà cao 30 m
        Cylinder(center_xy=(0, 0), radius=0.3),  # cột vô hạn
        Sphere(center=(20, -6, 10), radius=1.5),
    ]
)
rho, n_hat = obstacles.distance(traj.p[-1])  # (N, M) m  và  (N, M, 3)
```

- ρ là khoảng cách có dấu **chính xác** tới bề mặt: dương ở ngoài, âm ở trong. n̂ = ∇ρ
  là vector đơn vị hướng **ra xa** vật cản, nên lực đẩy cùng chiều +n̂.
- Trụ thẳng đứng có thể có nắp (`z_min`, `z_max` hữu hạn) hoặc hở một/hai đầu (`±inf`).
- Tại tâm cầu hoặc trên trục trụ, gradient không xác định. Khi đó n̂ được cố định là
  `+e_z` (cầu) và `+e_x` (trụ), nên lực đẩy không bao giờ mất hẳn (D003).
- Cột j của kết quả ứng với `obstacles[j]` theo đúng thứ tự khai báo.

### 7.3 Demo

```powershell
uv run python scripts/demo.py
```

Demo cho 5 UAV bay bằng bộ PD thuần qua một tòa nhà và một khối cầu. Script in ra
lực và tốc độ lớn nhất (đúng bằng giới hạn) và khe hở nhỏ nhất tới vật cản. Khe hở
nhỏ nhất chỉ khoảng 4 cm, vì PD không có cơ chế tránh.

### 7.4 Bản base APF (T1–T4)

```powershell
uv run python scripts/base_demo.py
```

`uavapf.control.APFController` ghép lực hút, lực đẩy chuẩn hóa theo đích, Morse +
barrier, giảm chấn Hessian chiếu PSD và lực con quay từ `paper/main.tex` (4)–(15).
`StallReset` thực hiện reset vận tốc (16) khi điều kiện kẹt được giữ đủ lâu; truyền
nó qua tham số `reset=` của `simulate`. Demo 3 UAV lưu quỹ đạo vào
`results/base_demo/apf_swarm.npz` hoặc `UAVAPF_RESULTS` nếu đã đặt biến môi trường.

Controller hiện hỗ trợ cầu và trụ đứng vô hạn. Trụ có nắp cần công thức Hessian
riêng. Controller yêu cầu trạng thái ban đầu ngoài vật cản và khoảng cách giữa
mọi cặp lớn hơn `d_min`; nếu vi phạm sẽ báo `ValueError`. Demo chạy không bão hòa
để phù hợp giả thiết định lý an toàn của bài báo. Nếu đặt `a_max` quá thấp, kết quả
mô phỏng có thể va chạm dù bộ điều khiển tính đúng công thức; định lý của bài chỉ
bao trùm trường hợp bão hòa không kích hoạt trên miền quỹ đạo đạt được.

### 7.5 Kịch bản YAML và chạy một lệnh (T05)

Định dạng file: `docs/specs/T05-scenarios-yaml.md`, ví dụ `configs/scenarios/demo_sphere.yaml`.

```python
from uavapf.scenarios import load_scenario
from uavapf.scenarios.run import run_scenario

scn = load_scenario("configs/scenarios/demo_sphere.yaml")
res = run_scenario(scn, "results/scenarios/demo_sphere")  # trajectory.npz, xy.png, anim.gif
print(res.collided, res.min_clearance, res.final_slot_error)
```

Trình đọc kiểm tra rất chặt: gõ sai tên khóa (ví dụ `raduis`) sẽ báo lỗi ngay, không
âm thầm dùng giá trị mặc định. Lượt chạy dừng ở lần va chạm đầu tiên (D005).

### 7.6 Sinh kịch bản S1–S5 theo seed (T13, bản nháp)

```python
from uavapf.scenarios import make_scenario, save_scenario

scn = make_scenario("S3", n=10, seed=42, formation="wedge", variant="u")
save_scenario(scn, "configs/scenarios/s3_u_n10_s42.yaml")  # chạy lại bằng run_scenario.py
```

S1 rừng trụ (`sparse` / `medium` / `dense`), S2 slot nằm trong vùng ảnh hưởng, S3
vật cản trên đường thẳng (`line`) hoặc chữ U lõm (`u`), S4 hành lang hẹp hơn đội hình,
S5 hai nhóm đối đầu (`groups`) hoặc hoán đổi vị trí (`swap`). Đội hình `line`, `wedge`,
`grid`. Mọi tham số hình học nằm trong `GeneratorParams` (đang chờ chốt, xem plan Q4).

### 7.7 Baseline B1 và chỉ số đánh giá (T06, T12)

```python
from uavapf.baselines import KhatibController
from uavapf.metrics import evaluate_run
from uavapf.sim import simulate

b1 = KhatibController(scn.slots, scn.obstacles)
traj = simulate(b1, scn.p0, scn.v0, scn.sim)
m = evaluate_run(traj, scn.slots, scn.obstacle_set(), d_min=0.5, eps_p=0.3, a_max=scn.sim.a_max)
print(m.as_dict())  # success, collided, completion_time, min_separation, ..., rms_jerk
```

---

## 8. Thư mục kết quả (`UAVAPF_RESULTS`)

Kết quả mô phỏng **không bao giờ** nằm trong repo (D001). Mọi script lấy thư mục gốc
từ `uavapf.paths.results_root()`:

- Có biến môi trường `UAVAPF_RESULTS`: dùng giá trị đó (hỗ trợ `~` và đường dẫn tương đối).
- Không có: dùng `.\results\` (đã được `.gitignore`).

Đặt cố định sang ổ HDD trên Windows:

```powershell
setx UAVAPF_RESULTS "D:\uav_results"
# mở cửa sổ mới để biến có hiệu lực
```

Chỉ đặt cho phiên hiện tại: `$env:UAVAPF_RESULTS = "D:\uav_results"`.

---

## 9. Quy trình Claude + Codex

Chi tiết nằm trong `AGENTS.md`. Tóm tắt:

1. **Một nhiệm vụ = một thay đổi nhỏ, dễ review** (mục tiêu dưới khoảng 400 dòng).
2. **Người viết test khác người viết code.** Mặc định: Claude viết spec
   (`docs/specs/`) và test (`tests/`), Codex viết code, Claude review. T09–T11 đảo vai.
3. Người cài đặt **không sửa `tests/`**. Nếu thấy test sai, ghi lý do vào
   `docs/handoff.md`.
4. Trước khi kết thúc phiên, AI nào cũng thêm một mục lên **đầu** `docs/handoff.md`.
5. Thông thường bạn tự commit sau khi `setup.cmd` báo `ALL DONE`. Agent chỉ commit hoặc
   push khi bạn yêu cầu tường minh cho lượt đó (D011).

> **Ngoại lệ hiện tại:** T01, T03 và T04 do Claude viết cả code lẫn test (theo yêu cầu
> trực tiếp). Vì vậy bước tiếp theo là để **Codex review cả ba**. Codex được phép
> **thêm** test nhưng không được sửa hay xóa test có sẵn.

Prompt giao cho Codex review (dán vào Codex, mở tại `C:\Setup\UAV`):

```
Read AGENTS.md, docs/handoff.md, docs/specs/T01-repo-setup.md, docs/specs/T03-sim.md
and docs/specs/T04-geometry.md.
Review the implementation in src/uavapf/ (paths.py, sim/, geometry/) and the tests in
tests/ against those specs. Check signs, units, RK4 stage formulas, saturation inside
every stage, signed-distance formulas for the capped cylinder (side, caps, rim corner,
inside), degenerate normals, and that tests would catch realistic bugs.
You may ADD new tests in new files tests/test_t0X_codex_*.py; do not edit existing tests.
Run uv run pytest -q and uv run ruff check . and report results.
List findings as blocking / should fix / nit with file:line, end with a verdict,
and add a handoff entry at the top of docs/handoff.md.
```

Bốn mẫu prompt cho vòng thường (spec → code → review → sửa) nằm ở mục 6 của kế hoạch.

---

## 10. Quy ước code

- Đơn vị SI (m, s, m/s, m/s²), ghi đơn vị trong docstring của mọi đầu vào/ra vật lý.
- Mảng `float64`. Vị trí/vận tốc có dạng `(N, 3)`.
- Hàm lực/thế là **hàm thuần**: không trạng thái ẩn, không biến toàn cục, không I/O.
- Ngẫu nhiên chỉ đi qua `np.random.Generator` tạo từ seed và truyền vào tường minh.
- Đường dẫn bằng `pathlib.Path`.
- Mọi công thức có comment trích nguồn: `# eq. (12), main.tex` hoặc trang của paper gốc.
- Type hint cho hàm public, docstring kiểu NumPy, dòng tối đa 100 ký tự (ruff).
- Code tính toán được vector hóa trên N UAV × M vật cản, không dùng vòng lặp Python
  theo từng UAV. Mỗi module có test so sánh bản vector hóa với vòng lặp.

---

## 11. Git

Remote GitHub: `https://github.com/tvtuan632004/uav-swarm-apf.git` (D011, thay D008).
Repo hiện công khai. Người dùng thường tự commit sau khi `setup.cmd` báo `ALL DONE`;
agent chỉ commit/push khi được yêu cầu tường minh.

Lần đầu:

```powershell
cd C:\Setup\UAV
git init -b main
git add .
git commit -m "Baseline"
```

Sau mỗi lượt của một AI (Claude hoặc Codex), bấm đúp `scripts\setup.cmd`; thấy
`ALL DONE` thì:

```powershell
git add .
git commit -m "T05 Codex: implement scenarios"      # mã nhiệm vụ + ai + việc
```

Xem AI vừa sửa gì: `git status`, `git diff`. Quay lại bản trước nếu cần:
`git restore <file>` (một file) hoặc `git reset --hard HEAD` (bỏ mọi thay đổi chưa commit).

`.github/workflows/ci.yml` sẽ chạy khi có sự kiện GitHub phù hợp.
File `.gitattributes` ép xuống dòng LF (riêng `.ps1`, `.cmd` dùng CRLF), để diff không
bị nhiễu khi chuyển giữa Windows và Ubuntu ở giai đoạn PX4.

---

## 12. Xử lý sự cố

| Triệu chứng | Cách xử lý |
|---|---|
| `running scripts is disabled on this system` | Chạy bằng `powershell -ExecutionPolicy Bypass -File scripts\setup.ps1` hoặc bấm đúp `setup.cmd` |
| `uv : The term 'uv' is not recognized` | Vừa cài uv xong: đóng và mở lại cửa sổ PowerShell. Vẫn lỗi: thêm `%USERPROFILE%\.local\bin` vào PATH |
| `uv sync --locked` báo lock lỗi thời | Có ai sửa `pyproject.toml` mà quên lock: chạy `uv lock` rồi commit `uv.lock` |
| `No Python at '"C:\...\python.exe'` khi chạy pytest | `.venv` trỏ tới một bản Python đã bị xóa hoặc chỉ tồn tại trong sandbox của AI khác. Chạy lại `setup.cmd` (script tự phát hiện và dựng lại), hoặc thêm `-Clean` |
| `PermissionError: [WinError 5] Access is denied: ...\Temp\pytest-of-Admin` | Thư mục tạm của pytest do sandbox của Codex tạo ra với quyền riêng. `setup.cmd` tự né lỗi này bằng thư mục `.pytest_tmp\`. Muốn sửa hẳn: mở **cmd bằng quyền Administrator** rồi chạy `takeown /f "%TEMP%\pytest-of-%USERNAME%" /r /d y`, `icacls "%TEMP%\pytest-of-%USERNAME%" /grant "%USERNAME%":F /t`, `rmdir /s /q "%TEMP%\pytest-of-%USERNAME%"` |
| `ModuleNotFoundError: uavapf` | Đang chạy `python` của conda. Dùng `uv run python ...` hoặc `conda deactivate` |
| `ruff format --check` báo lỗi | `uv run ruff format .` rồi commit |
| Test lỗi chỉ trên Windows | Thường do đường dẫn hoặc xuống dòng. Kiểm tra dùng `pathlib` và `.gitattributes` |
| Repo nằm trong OneDrive và chạy chậm | Để repo ngoài OneDrive (như `C:\Setup\UAV`), vì `.venv` có hàng nghìn file nhỏ |

Gửi `setup-report.txt` cho Claude khi cần chẩn đoán lỗi cài đặt.
