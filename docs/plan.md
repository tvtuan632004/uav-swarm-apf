# Kế hoạch code và thực nghiệm (bản 2.1): Claude + Codex

Cập nhật sáng 11/10/2026, sau đêm làm việc của Claude. Thay thế bản 2 (01:20 cùng ngày).
File này là nguồn chuẩn cho bảng nhiệm vụ; AGENTS.md trỏ về đây.

## 1. Tình hình hiện tại

- Repo `C:\Setup\UAV`, **chỉ dùng git trên máy, không GitHub** (D008). Kiểm tra: 295 test
  xanh trên Linux (bản sao của Claude); trên Windows lần cuối 111/111 trước đêm nay,
  cần bạn chạy lại `setup.cmd`.
- Xong trong đêm (Claude viết cả spec, test và code, **Codex cần review**):
  T11b (va chạm là kết quả), T05 (YAML, chạy một lệnh, hình, GIF), T11c (controller
  nhanh gấp 6 lần), T12 (7 chỉ số), T06 (baseline B1 Khatib, tái hiện GNRON và kẹt),
  T13 bản nháp (sinh S1–S5 theo seed), và một **pilot** APF so với B1 (168 lượt chạy).
- Pilot (`docs/pilot-2026-10-11.md`): khi không bão hòa, APF hơn B1 rõ ở S2, S3, S4, S5;
  với a_max = 5 m/s² gần như mọi lượt đều va chạm, do slot đứng yên ở xa làm bầy bay
  ~10 m/s. **Cần quyết Q1 (mục 6) trước khi làm tiếp T13–T18.**

## 2. Vòng làm việc

Mỗi nhiệm vụ đi qua 6 bước. Hai AI **không bao giờ làm cùng lúc** trong thư mục.

| Bước | Ai | Việc |
|---|---|---|
| 1 | Claude | Viết spec `docs/specs/Txx-*.md` và test `tests/test_txx_*.py`; kiểm test bằng bản cài đặt nháp và lỗi gài thử; chép sang máy |
| 2 | Bạn | `git add . && git commit -m "Txx spec+tests"` |
| 3 | Codex | Cài đặt trong môi trường riêng (`.venv-codex`), test xanh, ghi handoff |
| 4 | Bạn | Bấm đúp `scripts\setup.cmd`; thấy `ALL DONE` thì commit |
| 5 | Claude | Review: kéo file về, chạy test, đối chiếu `main.tex`, gài lỗi thử, ghi handoff |
| 6 | Codex → Bạn | Codex sửa theo review; bạn chạy `setup.cmd` và commit |

Khi một bên vừa viết test vừa viết code (đêm nay: T05, T06, T11b, T11c, T12, T13 đều do
Claude), bên kia **review trước khi coi là xong**, và được thêm test mới.

## 3. Bảng nhiệm vụ

Trạng thái: ✅ xong · 🔶 xong code, chờ review chéo · 📝 spec+test sẵn · ⬜ chưa làm · ❓ chờ quyết định.

| Mã | Nhiệm vụ | Spec+test | Code | Review | Xong khi | Trạng thái |
|---|---|---|---|---|---|---|
| T01 | Repo, uv, cấu trúc, setup.cmd | Claude | Claude | Codex | `setup.cmd` ra ALL DONE trên Windows | 🔶 |
| T02 | AGENTS.md, CLAUDE.md, handoff, D007 | Claude | Claude | Codex | Codex nêu lại đúng quy trình | ✅ |
| T03 | Mô phỏng tích phân kép, RK4, bão hòa, log | Claude | Claude | Codex | Năng lượng sai < 1e-6 (đạt 9e-11) | 🔶 |
| T04 | Vật cản cầu, trụ (có nắp): ρ, n̂, clearance | Claude | Claude | Codex | n̂ khớp gradient sai phân | 🔶 |
| T05 | Kịch bản YAML, chạy một lệnh, hình, GIF | Claude | Claude | Codex | `run_scenario.py demo_sphere.yaml` ra npz, png, gif | 🔶 |
| T06 | Baseline B1 Khatib | Claude | Claude | Codex | Tái hiện GNRON (lệch 0,44 m) và kẹt (x = −1,35 m) | 🔶 |
| T07–T11 | APF T1–T4 + reset (`control/apf.py`, `reset.py`) | Claude (test) | Codex | Claude | Gradient, PSD 10.000 trạng thái, vᵀf = 0, V không tăng | ✅ |
| T11b | Va chạm là kết quả (D005), metrics an toàn | Claude | Claude | Codex | Hoán đổi với a_max = 5 chạy hết, báo va chạm | 🔶 |
| T11c | Controller thưa (chỉ cặp đang tác động) | Claude | Claude | Codex | N=50, M=20 ≤ 1 ms (đạt 0,38–0,44 ms) | 🔶 |
| T12 | 7 chỉ số đánh giá | Claude | Claude | Codex | Khớp tính tay | 🔶 |
| T13 | Sinh S1–S5 theo seed (bản nháp v0) | Claude | Claude | Codex | Cùng seed cùng kịch bản; mỗi S có đúng khuyết tật | 🔶 ❓ tham số |
| **T13b** | **Tâm đội hình di chuyển (Remark tracking), nếu chọn Q1(a)** | Claude | Codex | Claude | a_max = 5 không còn va chạm hàng loạt trong pilot | ❓ Q1 |
| T14 | Baseline B2 Olfati-Saber | Claude | Codex | Claude | Tái hiện lưới α như bài gốc | ⬜ |
| T15 | B3 (Hessian đầy đủ), B4 (HOCBF-QP, OSQP), B5 (ORCA) | Claude | Codex | Claude | Cả ba chạy trên S1–S5 không lỗi | ⬜ |
| T16 | Runner Monte Carlo (từ `scripts/pilot.py`) + lưu config, seed, commit | Claude | Codex | Claude | 500 lượt tái lập từ một lệnh | ⬜ |
| T17 | Tinh chỉnh (Optuna), cùng ngân sách cho baseline | Claude | Codex | Claude | `configs/final.yaml` | ⬜ |
| T18 | Thực nghiệm chính, ablation A1–A4, quét a_max (D006) | Claude | Codex chạy | Claude | Bảng kết quả CI 95% | ⬜ |
| T19 | Hình, bảng, Results trong `main.tex` | Claude | Codex xuất hình | Claude viết | Bài biên dịch với số thật | ⬜ |
| T20 | ROS 2 + PX4 SITL (sau khi cài Ubuntu) | Claude | Codex | Claude | 5 UAV qua S1 trong Gazebo | ⬜ |

## 4. Lịch (tới mốc 31/1/2027) — đang đi trước khoảng 1,5 tuần

| Tuần | Ngày | Việc | Mốc kiểm tra |
|---|---|---|---|
| 1 | 12–18/10 | Quyết Q1–Q4; Codex review đêm 11/10; T13b nếu chọn Q1(a); chốt tham số T13 | Pilot với a_max = 5 không còn va chạm hàng loạt |
| 2–3 | 19/10–1/11 | T14 (B2), T15 phần B3 | |
| 4–5 | 2–15/11 | T15 phần B4, B5 | Đủ 5 baseline |
| 6 | 16–22/11 | T16 (Monte Carlo) | 500 lượt tái lập |
| 7–8 | 23/11–6/12 | T17 (tinh chỉnh) | `configs/final.yaml` |
| 9–11 | 7–27/12 | T18 (thực nghiệm, ablation, quét a_max) | Bảng kết quả + CI 95% |
| 12–13 | 28/12–10/1 | T19 (viết Results) | Bài biên dịch với số thật |
| 14–16 | 11–31/1 | Vùng đệm, sửa theo góp ý thầy; mốc quyết định 31/1 | |
| sau 1/2 | | T20 PX4 trên Ubuntu dual-boot | |

Nếu trễ: cắt B5 trước, rồi thu hẹp ablation. Mốc 31/1 như đề cương.

## 5. Quyết định đã chốt (`docs/decisions.md`)

- **D004** Bão hòa theo chuẩn (norm) mỗi UAV.
- **D005** Va chạm là kết quả: controller kẹp ρ và r − d_min ở 1 mm; metrics ghi nhận.
- **D006** Hai chế độ bão hòa + quét a_max ∈ {2, 5, 10, ∞}.
- **D007** Luân phiên; Codex dùng `.venv-codex`, `.pytest_tmp/codex`.
- **D008** Không dùng GitHub: git trên máy; xác nhận bằng `setup.cmd` (Windows) và bản
  chạy Linux của Claude. `.github/workflows/ci.yml` giữ lại nhưng không bắt buộc.

## 6. Câu hỏi cần bạn quyết (chi tiết: `docs/pilot-2026-10-11.md`)

1. **Q1 — tốc độ tiếp cận.** (a) Tâm đội hình di chuyển với tốc độ giới hạn (đề xuất);
   (b) rút ngắn quãng đường; (c) thêm giới hạn tốc độ v_max.
2. **Q2 — bước tích phân.** Nếu chọn (a) thì dt = 0,01 s có lẽ đủ; nếu không, giảm dt.
3. **Q3 — B1 đẩy cặp theo r (cổ điển) hay r − d_min (công bằng hơn với tiêu chí d_min).**
4. **Q4 — tham số T13** (mật độ rừng 3/6/10 cây/100 m², hành lang 50 % bề rộng, L = 20 m).

## 7. Việc của bạn sáng nay

1. Bấm đúp `scripts\setup.cmd` → mong đợi **295 passed**, demo, `ALL DONE`.
2. Commit:
   ```
   cd C:\Setup\UAV
   git init -b main          (nếu chưa)
   git add .
   git commit -m "Night 11/10: T05 T06 T11b T11c T12 T13-draft, pilot"
   ```
3. Xem `results\scenarios\demo_sphere\anim.gif` (chạy
   `uv run python scripts\run_scenario.py configs\scenarios\demo_sphere.yaml`).
4. Trả lời Q1–Q4. 5. Giao Codex review (prompt mục 8).

## 8. Mẫu prompt cho Codex

Mở đầu chung:
```
Read AGENTS.md (especially section 5), docs/plan.md, the newest entry in
docs/handoff.md and the spec named below. Set UV_PROJECT_ENVIRONMENT=.venv-codex and
PYTEST_DEBUG_TEMPROOT=.pytest_tmp/codex before any uv or pytest command.
```

Review công việc đêm 11/10 của Claude (làm việc này trước):
```
Claude wrote spec, tests AND code for T05, T06 (B1), T11b, T11c, T12 and T13 overnight.
Review each against its spec in docs/specs/ (T05-scenarios-yaml, T06-b1-khatib,
T11b-contact, T11c-speed, T12-metrics, T13-scenarios) and paper/main.tex. Check the
Khatib FIRAS form against the original paper. You may ADD tests in new files
tests/test_*_codex.py, never edit existing ones. Findings as blocking / should fix /
nit with file:line, a verdict per task, then a handoff entry.
```

Cài đặt (bước 3) và sửa theo review (bước 6): như bản 2 — `Implement docs/specs/<Txx>.md
exactly …` / `Fix every "blocking" and "should fix" item …`; xong = 0 failed, 0 skipped,
ruff sạch, có mục handoff.

## 9. Bài học

1. Hai AI cùng ghi một thư mục thì ghi đè lẫn nhau → luân phiên + git.
2. Sandbox của Codex làm hỏng `.venv` và thư mục tạm của bạn → D007, `setup.cmd` tự sửa.
3. Test của việc chưa làm phải tự bỏ qua (`importorskip`); xong = 0 failed, 0 skipped.
4. Kiểm công thức chưa đủ: phải chạy ca xấu nhất. Pilot cho thấy tham số trong bài
   (a_max, slot đứng yên ở xa) làm định lý không áp dụng được ở quy mô thật.
