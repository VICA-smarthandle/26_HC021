# 앱 '지도 수정'(지우기 · 부분 스캔) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 관리자가 앱에서 지도의 변경된 곳만 지우거나(사각형 여러 개) 정지 상태 라이다 스캔으로 다시 그려(반경 1·2·3 m) Nav2 재시작 없이 반영한다.

**Architecture:** 판단은 순수 파이썬 모듈 셋(`map_fix_store`·`map_fix_scan`·`map_fix_session`), ROS 배선은 새 노드 `map_fix_node` 하나가 맡는다(금지구역의 `keepout_mask` + `keepout_map_node` 와 같은 구조). 앱은 서비스 `/vica/map_fix` 하나와 토픽 `/vica/map_fix/state` 하나로 말한다.

**Tech Stack:** Python 3.10 · numpy · scipy(`vica_nav2.pose_score` 재사용) · rclpy(Humble) · Flutter(앱).

**Spec:** `docs/superpowers/specs/2026-11-18-map-fix-erase-partial-scan-design.md`(이하 설계서). 화면·문구 정본은 캔버스 75~83번.

## Global Constraints

- 작업 폴더: ROS `~/wt-map-fix`(브랜치 `feat/map-fix`), 앱 `~/wt-map-fix-app`(브랜치 `feat/map-fix`). 운영 폴더
  `/home/ji_w/VICA-smarthandle/{vica_ros2_ws,VICA_Supervisor}` 는 고치지 않는다.
- **로봇이 살아 있다.** 다른 세션의 실기 중에는 `colcon`·`flutter`·`ros2` 명령, 유닛 재시작, 운영 `maps/` 쓰기,
  `load_map` 호출을 하지 않는다. 이 조건이 걸린 단계는 제목에 `[잠금]` 을 붙였다 — 사용자 확인 뒤에만 한다.
- 순수 모듈 시험은 `nice -n 19 python3 -m pytest -q <파일>` 로 돌린다(rclpy 초기화 없음).
- 지도 값: 검은 칸 0 · 빈칸 254 · 회색 칸 205. 지도 크기·origin·resolution 은 바꾸지 않는다.
- 좌표는 ROS map 좌표(m). `map_id` 는 `keepout_mask.validate_map_id` 로 검사한다.
- 칸 순서 두 가지를 섞지 않는다: **그림 순서**(pgm, 0행 = 위 = y 큰 쪽)와 **격자 순서**(`pose_score.MapGrid`,
  0행 = `origin_y` 쪽). 바꾸는 식은 `격자행 = height − 1 − 그림행` 하나다.
- 앱 글자는 83번 보드 그대로. 새 글자는 `vicaKeepWords` 를 거친다. 옛 말 '지도 고치기'·'다시 그리기'·'고침' 금지.
- 주석·문서 문체는 각 저장소의 기존 문체(앱 저장소 `ros2/` 는 '~합니다', `vica_nav2` 는 '~다').
- 노드는 파이썬으로 만들되 쉬는 동안 가볍게 둔다(11-19 사용자 "파이썬으로 진행"): `map_fix_node.py` 는 모듈
  맨 위에서 numpy·scipy·`map_fix_scan`·`map_fix_session`·`vica_nav2` 를 import 하지 않고 요청을 처리하는 함수
  안에서 처음 쓸 때 import 한다. `/scan`·`/odom`·TF 구독은 스캔 세션과 `scan_check` 동안만 연다. 구현 뒤 쉬는
  동안의 CPU·메모리를 재서 사용자에게 알린다(비교값: `map_list_node.py` 46 MB, `keepout_map_node.py` 60 MB).
- 커밋은 작업 폴더의 `feat/map-fix` 에만, 푸시하지 않는다. 메시지 끝에
  `Co-Authored-By: Claude Fable 5.2 <noreply@anthropic.com>`.

## Review Focus

1. 지도 파일이 예상과 다르다(P2 형식·maxval ≠ 255·yaml 의 `image` 가 다른 이름·png 없음) → 죽지 않고 `bad_map`
   또는 `no_map` 으로 거절한다. (Task 1)
2. 수정 도중 전원이 나간다 → 임시 파일에 쓴 뒤 이름을 바꾸고 백업이 먼저라, 다시 켰을 때 pgm 이 온전하다.
   `entry.json` 없는 반쪽 백업 폴더는 기록·되돌리기에서 무시한다. (Task 2)
3. 로봇이 지도 가장자리에 있어 원이 지도 밖으로 나가거나, 트인 곳이라 반경 안에 맞는 빔이 거의 없다 → 지도
   밖은 잘라내고 `no_change` 또는 `low_match` 로 끝난다(배열 범위 오류 금지). (Task 3·4)
4. 앱 두 대가 동시에 누른다·세션 중 앱이 끊긴다 → 요청은 한 번에 하나, 스캔 세션은 하나(`busy_session`),
   미리보기는 5분 뒤 스스로 버린다. (Task 5·8)
5. 스캔을 모으는 동안 누가 로봇을 민다·주행 요청이 들어온다 → 즉시 `moved`/`busy_driving` 으로 끝내고 지도를
   건드리지 않는다. (Task 5·8)

---

## File Structure

앱 저장소 `~/wt-map-fix-app/ros2/`
| 파일 | 책임 |
|---|---|
| `map_fix_store.py`(새) | 지도 파일 읽기·쓰기, 사각형 → 칸, 지우기 계산, 백업·기록·되돌리기, 그림 ↔ 격자 변환 |
| `map_fix_scan.py`(새) | 자세 다시 맞춤·구분 검사, 스캔 누적, 변경 판정 |
| `map_fix_session.py`(새) | 부분 스캔 한 번의 단계 진행(맞춤 → 물러나기 → 스캔 → 미리보기), ROS 없음 |
| `map_fix_node.py`(새) | 서비스·토픽·구독·`load_map`·잠금 판정 |
| `test_map_fix_store.py`·`test_map_fix_scan.py`·`test_map_fix_session.py`·`test_map_fix_node.py`(새) | 위 넷의 시험 |
| `map_list_node.py`(수정) | `/map_list` 에 `image_version`, 지도 삭제 때 `.fix_backup/<id>/` |
| `supervisor_bringup.launch.py`(수정) | `map_fix_node` 띄우기 |

ROS 저장소 `~/wt-map-fix/src/`
| 파일 | 책임 |
|---|---|
| `vica_interfaces/srv/MapFix.srv`(새) · `vica_interfaces/CMakeLists.txt`(수정) | 공용 계약 |
| `vica_nav2/vica_nav2/pose_bootstrap_node.py`(수정) | 지도 내용이 바뀌면 거리표 다시 만들기 |

저장소 밖 `~/vica_data/map_fix_tests/2026-11-19_dock/`: 0단계 실측 스크립트와 기록(지우지 않는다).

---

### Task 1: `map_fix_store` — 지도 읽기·쓰기와 지우기 계산

**Files:**
- Create: `~/wt-map-fix-app/ros2/map_fix_store.py`
- Test: `~/wt-map-fix-app/ros2/test_map_fix_store.py`

**Interfaces:**
- Consumes: `keepout_mask as km` — `km.MapMeta`, `km.read_map_meta(maps_root, map_id)`, `km.validate_map_id`,
  `km.read_map_yaml`, `km.write_all_atomic(targets)`, `km.KeepoutError(reason, message)`.
- Produces:
```python
OCCUPIED, FREE, UNKNOWN = 0, 254, 205
MAX_RECTS = 10   # 11-19 사용자(처음 20)
MIN_RECT_M = 0.05

class MapFixError(Exception):            # .reason, .message — 서비스 응답에 그대로 싣는다
    def __init__(self, reason: str, message: str): ...

@dataclass(frozen=True)
class MapImage:
    meta: km.MapMeta
    pixels: np.ndarray                   # (height, width) uint8, 그림 순서
    free_thresh: float                   # yaml 값, 없으면 0.25
    occupied_thresh: float               # yaml 값, 없으면 0.65

def load_map_image(maps_root: Path, map_id: str) -> MapImage
def normalize_rects(raw: list) -> list[dict]          # x_min<x_max 로 정리, bad_rect·too_small·too_many
def rect_cells(meta: km.MapMeta, rect: dict) -> tuple[slice, slice] | None   # (그림행, 열), 전부 밖이면 None
def count_erase(image: MapImage, rects: list[dict]) -> dict
    # {"rects": [{"w_m", "h_m", "black", "gray"}], "black": int, "gray": int} — 합계는 겹친 칸을 한 번만 센다
def apply_erase(image: MapImage, rects: list[dict]) -> tuple[np.ndarray, dict]   # (새 pixels, count_erase 와 같은 dict)
def encode_pgm(pixels: np.ndarray) -> bytes           # P5, maxval 255
def encode_png(pixels: np.ndarray) -> bytes           # 8비트 회색, zlib 만 사용(PIL 없음)
def write_map_pixels(maps_root: Path, map_id: str, pixels: np.ndarray) -> None   # pgm·png 를 km.write_all_atomic 으로
def to_grid(image: MapImage) -> "pose_score.MapGrid"  # Nav2 trinary 규칙, 격자 순서, 값 100/0/-1
def cells_to_pixels(cells: np.ndarray, height: int) -> np.ndarray   # 격자 (행, 열) → 그림 (행, 열)
```
- 칸 고르는 규칙: 칸 중심 `origin + (i + 0.5)·resolution` 이 사각형 안(경계 포함)이면 든다. 지도 밖은 잘라낸다.
- `to_grid` 의 규칙: `occ = (255 − 픽셀)/255`; `occ > occupied_thresh` → 100, `occ < free_thresh` → 0, 나머지 −1
  (Nav2 `map_io.cpp` 와 같다. 지금 yaml 로는 205 가 0 이 된다).

- [ ] **Step 1: 시험을 먼저 쓴다** (`test_map_fix_store.py`, `test_keepout_mask.make_map` 을 가져다 쓴다 — 10×8 칸,
  resolution 0.1, origin (−0.5, −0.4), 전부 254)

```python
def test_cell_centres_decide_membership(tmp_path):
    # 사각형 x 0.00~0.20, y 0.00~0.10 → 칸 중심 0.05·0.15 (열 5·6), y 0.05 (격자행 4 → 그림행 3)
    rows, cols = store.rect_cells(image.meta, {"x_min": 0.0, "y_min": 0.0, "x_max": 0.2, "y_max": 0.1})
    assert (rows, cols) == (slice(3, 4), slice(5, 7))

def test_rect_is_clipped_to_the_map_and_fully_outside_is_none(tmp_path): ...
    # x −9~0.0 → 열 0~4 로 잘림 ; x 5~6 → None

def test_counts_black_and_gray_and_totals_count_overlap_once(tmp_path):
    # 픽셀 [3,5]=0, [3,6]=205 로 고친 뒤 겹치는 사각형 둘 → rects[0] black 1 gray 1, rects[1] black 1, 합계 black 1 gray 1

def test_apply_erase_sets_only_selected_cells_to_free(tmp_path):
    # 고른 칸만 254, 나머지 바이트 동일

def test_rect_errors(tmp_path):
    # 21개 → too_many, 한 변 0.04 → too_small, {"x_min": "a"} → bad_rect, 전부 지도 밖 → out_of_map(apply·count 둘 다)

def test_pgm_png_roundtrip_and_files_written_atomically(tmp_path):
    # write_map_pixels 뒤 load_map_image(...).pixels 가 같고, png 는 IHDR 폭·높이·8비트 회색, maps 폴더에 임시 파일이 남지 않는다

def test_to_grid_follows_nav2_trinary_rule(tmp_path):
    # 픽셀 0/205/254 → 100/0/0 (free_thresh 0.25), yaml free_thresh 0.196 이면 205 → −1 ; 0행이 origin_y 쪽

def test_bad_map_files_are_rejected_not_crashed(tmp_path):        # Review Focus 1
    # P2 pgm, maxval 65535, 잘린 pgm, yaml 없음 → MapFixError("bad_map" | "no_map")
```

- [ ] **Step 2: 실패 확인** — `cd ~/wt-map-fix-app/ros2 && nice -n 19 python3 -m pytest -q test_map_fix_store.py` → `ModuleNotFoundError: map_fix_store`
- [ ] **Step 3: `map_fix_store.py` 를 위 Interfaces 대로 구현한다.** pgm 은 P5·maxval 255 만 받는다. png 는
  `zlib.compress` 와 `zlib.crc32` 로 IHDR(색 종류 0, 비트 8)·IDAT·IEND 를 직접 쓴다. `km.KeepoutError` 는
  `MapFixError` 로 바꿔 올린다(reason 유지).
- [ ] **Step 4: 통과 확인** — 같은 명령 → 전부 PASS
- [ ] **Step 5: Commit** — `git -C ~/wt-map-fix-app add ros2/map_fix_store.py ros2/test_map_fix_store.py && git -C ~/wt-map-fix-app commit -m "feat(ros2): map_fix_store — 지도 읽기·쓰기와 지우기 계산"`

### Task 2: `map_fix_store` — 백업 · 기록 · 되돌리기

**Files:**
- Modify: `~/wt-map-fix-app/ros2/map_fix_store.py`
- Test: `~/wt-map-fix-app/ros2/test_map_fix_store.py`

**Interfaces:**
- Produces:
```python
BACKUP_DIRNAME = ".fix_backup"
KEEP_BACKUPS = 10

def backup_root(maps_root: Path, map_id: str) -> Path                 # maps/.fix_backup/<map_id>
def save_backup(maps_root: Path, map_id: str, kind: str, entry: dict, now: datetime) -> Path
    # 지금의 pgm·png 를 <YYYYMMDD_HHMMSS>_<kind>/ 에 복사하고 entry.json 을 맨 마지막에 쓴다. kind ∈ {"erase","scan"}.
    # 같은 초에 또 부르면 이름 뒤에 _2. 끝나면 오래된 것부터 지워 KEEP_BACKUPS 개만 남긴다. 돌려주는 값은 그 폴더.
def list_history(maps_root: Path, map_id: str, limit: int = 3) -> list[dict]   # 최근 것부터, entry.json 내용
def restore_backup(maps_root: Path, map_id: str, backup: Path) -> None          # 그 폴더의 pgm·png 를 원자적으로 되살린다
def undo_last(maps_root: Path, map_id: str) -> dict   # 가장 최근 것을 되살리고 그 폴더를 지운다. 없으면 MapFixError("nothing_to_undo", …)
def drop_backup(backup: Path) -> None                 # 수정이 실패해 백업이 필요 없어졌을 때
```
- `entry.json` 키: `at`(KST ISO 초 단위) · `kind` · `radius_m` · `added` · `removed` · `black` · `gray` · `rects` · `pose`.
  없는 값은 `null`.

- [ ] **Step 1: 시험을 먼저 쓴다**

```python
def test_backup_then_undo_restores_bytes_exactly(tmp_path):
    # 지우기 → write_map_pixels → undo_last 뒤 pgm·png 바이트가 처음과 같다. 백업 폴더는 사라진다.
def test_history_is_newest_first_and_limited(tmp_path):        # 4번 저장 → list_history(limit=3) 의 at 이 내림차순 3개
def test_only_ten_backups_are_kept(tmp_path):                  # 12번 저장 → 폴더 10개, 가장 오래된 둘이 없다
def test_same_second_gets_a_suffix(tmp_path):                  # now 가 같은 두 번 → …_erase, …_erase_2
def test_nothing_to_undo(tmp_path):                            # MapFixError.reason == "nothing_to_undo"
def test_half_written_backup_is_ignored(tmp_path):             # Review Focus 2: entry.json 없는 폴더는 기록·되돌리기에서 빠진다
def test_restore_leaves_original_when_backup_is_broken(tmp_path):   # 백업의 pgm 이 없으면 MapFixError("io_error"), 지금 파일은 그대로
```

- [ ] **Step 2: 실패 확인** — `nice -n 19 python3 -m pytest -q test_map_fix_store.py -k "backup or undo or history"` → FAIL(`AttributeError`)
- [ ] **Step 3: 구현.** 폴더 이름의 시각으로 정렬한다(mtime 아님). 복사는 `shutil.copy2`, 되살리기는 `km.write_all_atomic`.
- [ ] **Step 4: 통과 확인** — `nice -n 19 python3 -m pytest -q test_map_fix_store.py` → 전부 PASS
- [ ] **Step 5: Commit** — `feat(ros2): map_fix_store — 백업·기록·되돌리기`

### Task 3: `map_fix_scan` — 스캔 누적과 변경 판정

**Files:**
- Create: `~/wt-map-fix-app/ros2/map_fix_scan.py`
- Test: `~/wt-map-fix-app/ros2/test_map_fix_scan.py`

**Interfaces:**
- Consumes: `vica_nav2.pose_score` — `MapGrid(data, resolution, origin_x, origin_y)`(격자 순서), `OCCUPIED_THRESHOLD = 65`,
  `MIN_RANGE = 0.25`. 시험은 `sys.path` 에 `~/wt-map-fix/src/vica_nav2` 를 넣어 가져온다(환경 변수 `VICA_NAV2_SRC` 가
  있으면 그 경로).
- Produces:
```python
HIT_RATIO = 0.8          # 새로 생김: 맞은 스캔 ÷ N
PASS_RATIO = 0.8         # 없어짐: 지나간 스캔 ÷ N
MAX_HIT_RATIO = 0.1      # 없어짐: 맞은 스캔 ÷ N 상한
NEIGHBOR_CELLS = 1       # 주변 몇 칸
BEYOND_M = 0.10          # 빔이 그 칸을 지나 이만큼 더 가야 '지나감'
RAY_STEP_M = 0.025       # 빔 위를 훑는 간격(반 칸)

class ScanAccumulator:
    def __init__(self, grid: MapGrid, radius_m: float) -> None
    def add_scan(self, ranges, angle_min: float, angle_increment: float,
                 pose: tuple[float, float, float], sensor: tuple[float, float, float]) -> None
        # pose = base 의 map 자세 (x, y, yaw[rad]), sensor = base 기준 라이다 (x, y, yaw[rad])
    n_scans: int
    hit_scans: np.ndarray    # (H, W) int32, 격자 순서 — 그 칸에 끝점이 떨어진 스캔 수
    pass_scans: np.ndarray   # (H, W) int32 — 빔이 BEYOND_M 넘게 더 나아가며 지나간 스캔 수
    in_radius: np.ndarray    # (H, W) bool — 칸 중심이 첫 스캔의 라이다에서 radius_m 안

@dataclass(frozen=True)
class Changes:
    added: np.ndarray        # (N, 2) int, 격자 (행, 열)
    removed: np.ndarray      # (M, 2) int

def detect_changes(grid: MapGrid, acc: ScanAccumulator) -> Changes
def patched_grid(grid: MapGrid, changes: Changes) -> MapGrid      # added → 100, removed → 0, 원본은 안 바꾼다
def cell_centres(grid: MapGrid, cells: np.ndarray) -> np.ndarray  # (K, 2) map 좌표 (x, y) — 앱에 보낼 값
```
- 한 스캔에서 한 칸은 '맞음'·'지나감'을 각각 최대 1번 센다. 끝점이 [0.25 m, radius_m] 밖이면 맞음이 아니다.
  radius_m 보다 먼 빔은 radius_m 까지만 지나간 것으로 본다. inf·nan·0.25 m 미만은 버린다.
- 판정(반경 안 칸만, N = `n_scans`, N = 0 이면 빈 Changes):
  새로 생김 = 검은 칸(≥65)이 아니고 · `hit_scans ≥ HIT_RATIO·N` · 주변 `NEIGHBOR_CELLS` 칸(8방향) 안에 검은 칸 없음.
  없어짐 = 검은 칸이고 · `pass_scans ≥ PASS_RATIO·N` · `hit_scans ≤ MAX_HIT_RATIO·N` · 주변 칸 안에 새로 생김 후보 없음.

- [ ] **Step 1: 시험을 먼저 쓴다** — 합성 지도는 `test_pose_score.py` 의 방식(전부 100 인 판에서 방을 0 으로 파냄),
  합성 스캔은 지도를 광선으로 훑는 작은 도우미 `cast(grid, pose, sensor, count=360)` 를 시험 파일에 둔다.

```python
def test_unchanged_room_reports_no_changes():
    # 4×4 m 방 한가운데, 같은 지도로 만든 스캔 20개 → added 0, removed 0
def test_new_box_is_added_at_its_true_cells():
    # '실제' 지도에만 0.4 m 상자를 넣어 스캔 → added 칸이 상자의 보이는 변에서 1칸 안, removed 0
def test_removed_box_is_cleared():
    # '지도'에만 상자 → removed 가 상자 칸(보이는 쪽), added 0
def test_passing_person_is_filtered_out():
    # 스캔 20개 중 5개에만 사람(0.3 m 원) → added 0 (5/20 < 0.8)
def test_standing_person_is_added():
    # 20개 전부에 사람 → added > 0
def test_wall_half_a_cell_off_is_not_redrawn():
    # 스캔 자세를 벽 쪽으로 0.025 m 밀어 넣어도 added 0, removed 0 (주변 1칸 규칙)
def test_grazing_beams_do_not_erode_a_wall():
    # 벽과 5° 로 나란한 자세에서 removed 0 (BEYOND_M 규칙)
def test_radius_limits_changes():
    # 상자 둘(1.5 m, 2.8 m), radius 2.0 → 가까운 것만 added
def test_circle_partly_outside_the_map_is_clipped():          # Review Focus 3
    # 로봇이 지도 모서리 0.3 m 안, radius 3.0 → 예외 없이 끝난다
def test_patched_grid_and_cell_centres():
    # patched_grid 가 원본을 안 바꾸고, cell_centres 가 origin + (열 + 0.5)·res 를 낸다
```

- [ ] **Step 2: 실패 확인** — `nice -n 19 python3 -m pytest -q test_map_fix_scan.py` → `ModuleNotFoundError: map_fix_scan`
- [ ] **Step 3: 구현.** 빔마다 `RAY_STEP_M` 간격 표본점을 numpy 로 한 번에 만들고, 스캔 한 장의 '지나간 칸'·'맞은 칸'을
  bool 판에 표시한 뒤 누적 판에 더한다(같은 칸을 여러 빔이 지나도 1). 주변 칸 검사는 `scipy.ndimage.binary_dilation`.
- [ ] **Step 4: 통과 확인** — 같은 명령 → 전부 PASS
- [ ] **Step 5: Commit** — `feat(ros2): map_fix_scan — 스캔 누적과 변경 판정`

### Task 4: `map_fix_scan` — 자세 다시 맞춤과 구분 검사

**Files:**
- Modify: `~/wt-map-fix-app/ros2/map_fix_scan.py`
- Test: `~/wt-map-fix-app/ros2/test_map_fix_scan.py`

**Interfaces:**
- Consumes: `pose_score.filter_beams`, `build_likelihood_field(grid) -> (H, W) m`, `search_pose(field, grid, beams, x, y, *,
  yaw_hint, sensor) -> MatchResult`, `beam_hits(beams, x, y, yaw, *, sensor) -> (wx, wy)`, `score_pose(...) -> 0~100`.
- Produces:
```python
KEEP_DIST_M = 0.10       # 1회째 자세에서 끝점이 검은 칸 이 거리 안이면 남긴다
MIN_KEPT = 60
MIN_KEPT_RATIO = 0.5
MAX_RMS_M = 0.03
FINE_XY_M = 0.05         # 2회째 탐색 범위
FINE_YAW = math.radians(1.0)
AMBIG_SHIFT_M = 0.10
AMBIG_YAW = math.radians(2.0)
AMBIG_NEAR_M = 0.05
AMBIG_KEEP = 0.8

@dataclass(frozen=True)
class RefineResult:
    ok: bool
    reason: str              # "" | "low_match" | "ambiguous"
    x: float; y: float; yaw: float
    score: float             # pose_score 점수(유효 빔 전부) — 화면의 '위치 맞춤 %'
    used_beams: int
    kept_beams: int
    rms_m: float
    moved_m: float           # 출발 자세에서 옮긴 거리
    moved_deg: float
    ambiguous: tuple[str, ...]   # "forward" | "lateral" | "yaw"
    hits: np.ndarray         # (K, 2) 맞춘 자세에서 본 라이다 점 map 좌표, 최대 180개

def interp_field(field: np.ndarray, grid: MapGrid, wx: np.ndarray, wy: np.ndarray, max_dist: float) -> np.ndarray
    # 칸 중심 사이 선형 보간. 지도 밖은 max_dist.
def refine_pose(grid: MapGrid, field: np.ndarray, ranges, angle_min: float, angle_increment: float,
                guess: tuple[float, float, float], sensor: tuple[float, float, float]) -> RefineResult
def pose_score_percent(grid: MapGrid, field: np.ndarray, ranges, angle_min: float, angle_increment: float,
                       pose: tuple[float, float, float], sensor: tuple[float, float, float]) -> float
    # 미리보기의 score_before·score_after 가 쓰는 같은 잣대
```
- 순서: `filter_beams` → `search_pose`(1회째) → 남길 빔 → 2회째(남긴 빔의 `interp_field` 거리 제곱 평균이 가장 작은
  자세를 ±`FINE_XY_M`·±`FINE_YAW` 에서 2단 격자로: 0.01 m·0.2° → 0.002 m·0.05°) → 통과 조건 → 구분 검사.
- `low_match`: 남긴 빔 < `MIN_KEPT`, 남긴 빔 ÷ `used_beams` < `MIN_KEPT_RATIO`, RMS > `MAX_RMS_M`, 2회째 1단의 최선이 범위
  끝에 걸림 중 하나. `search_pose` 가 `ok=False` 여도 그 자세에서 계속 간다(판정은 위 조건이 한다).
- 구분 검사: 축마다 양쪽으로 밀어 `interp_field ≤ AMBIG_NEAR_M` 인 남긴 빔 수가 민 뒤에도 원래의 `AMBIG_KEEP` 을
  **양쪽 다** 넘으면 그 축은 구분되지 않는다. forward·lateral 은 로봇 기준(yaw 방향·그 왼쪽).

- [ ] **Step 1: 시험을 먼저 쓴다**

```python
def test_refine_recovers_a_shifted_guess_in_a_room():
    # 6×5 m 방, 참값 (2.0, 1.5, 0.3), 출발 (2.06, 1.46, 0.3+1.5°) → |dx|,|dy| ≤ 0.01, |dyaw| ≤ 0.2°, ok
def test_refine_ignores_a_moved_object():
    # 지도엔 상자가 왼쪽, 스캔엔 오른쪽(빔의 15 %) → 자세 오차 ≤ 0.01 m, kept_beams < used_beams, ok
def test_long_corridor_is_ambiguous_forward():
    # 폭 2 m·길이 30 m 복도 한가운데(끝 벽이 11 m 밖) → ok False, reason "ambiguous", ambiguous == ("forward",)
def test_room_corner_is_not_ambiguous():
def test_mostly_unmatched_scan_is_low_match():
    # 스캔의 70 % 가 지도에 없는 물체 → reason "low_match"
def test_open_space_with_few_beams_is_low_match():           # Review Focus 3: 유효 빔 40개
def test_score_is_the_pose_score_percent():
    # RefineResult.score == pose_score.score_pose(field, grid, filter_beams(...), x, y, yaw, sensor=sensor) (±1e-6)
def test_interp_field_is_linear_between_cell_centres():
```

- [ ] **Step 2: 실패 확인** — `nice -n 19 python3 -m pytest -q test_map_fix_scan.py -k "refine or ambiguous or low_match or interp or score"` → FAIL
- [ ] **Step 3: 구현.** 격자 탐색은 후보 자세마다 남긴 빔 끝점을 한꺼번에 계산해 `interp_field` 를 한 번 부른다.
- [ ] **Step 4: 통과 확인** — `nice -n 19 python3 -m pytest -q test_map_fix_scan.py` → 전부 PASS
- [ ] **Step 5: Commit** — `feat(ros2): map_fix_scan — 자세 다시 맞춤과 구분 검사`

### Task 5: `map_fix_session` — 부분 스캔 한 번의 진행

**Files:**
- Create: `~/wt-map-fix-app/ros2/map_fix_session.py`
- Test: `~/wt-map-fix-app/ros2/test_map_fix_session.py`

**Interfaces:**
- Consumes: Task 3·4 의 `ScanAccumulator`·`detect_changes`·`patched_grid`·`cell_centres`·`refine_pose`·`pose_score_percent`,
  `pose_score.build_likelihood_field`.
- Produces:
```python
STEP_BACK_SEC = 5.0
SCAN_SEC = 5.0
PREVIEW_TIMEOUT_SEC = 300.0
MAX_CELLS = 2000
MOVING_LIN = 0.02        # m/s
MOVING_ANG = 0.05        # rad/s

class ScanSession:
    def __init__(self, session_id: str, map_id: str, grid: MapGrid, radius_m: float, now: float) -> None
    phase: str               # "refine" | "step_back" | "scanning" | "preview" | "failed" | "done"
    reason: str
    def on_scan(self, now: float, ranges, angle_min: float, angle_increment: float,
                guess: tuple[float, float, float], sensor: tuple[float, float, float]) -> None
        # refine 단계의 첫 스캔으로 자세를 맞추고, scanning 단계에서는 맞춘 자세로 누적한다
    def on_motion(self, linear: float, angular: float) -> None     # 문턱을 넘으면 refine·step_back·scanning 에서 failed("moved")
    def on_lock(self, locked: bool) -> None                        # 잠금이 걸리면 preview 전 단계에서 failed("busy_driving")
    def tick(self, now: float) -> None                             # 시간으로 넘어가는 단계 전환, preview 5분 뒤 done
    def cancel(self) -> None                                       # → done
    def state(self, now: float) -> dict                            # 토픽에 실을 dict(설계서 6.2)
    changes: Optional[Changes]                                     # preview 에서만
    pose: Optional[tuple[float, float, float]]
    score_before: float; score_after: float
```
- 스캔이 2 s 동안 한 장도 안 오면(refine·scanning) `failed("no_pose")`. 변경된 칸이 0 이면 `failed("no_change")`.
- `state()` 의 `cells` 는 `[[x, y, 1|-1], …]`, `MAX_CELLS` 를 넘으면 새로 생김부터 잘라 넣는다. `remaining_s` 는 올림 정수.

- [ ] **Step 1: 시험을 먼저 쓴다** (시각은 숫자로 넣는다)

```python
def test_happy_path_refine_stepback_scan_preview():
    # t=0 생성 → 첫 스캔(t=0.1) 뒤 phase "step_back" → tick(5.2) "scanning" → 스캔 30장 → tick(10.3) "preview",
    # state()["added"] > 0, score_after > score_before, cells 가 [[x,y,1],…]
def test_refine_failure_goes_to_failed_with_reason():      # 복도 장면 → failed, reason "ambiguous", state()["ambiguous"] == ["forward"]
def test_motion_during_scan_fails_with_moved():            # Review Focus 5
def test_lock_during_step_back_fails_with_busy_driving():  # Review Focus 5
def test_no_scan_for_two_seconds_fails_with_no_pose():
def test_no_change_fails_with_no_change():
def test_preview_times_out_after_five_minutes():           # Review Focus 4: tick(preview 시작 + 301) → "done"
def test_cancel_at_any_phase():
def test_remaining_seconds_count_down():                   # step_back 에서 t=0.1 → 5, t=2.6 → 3
def test_cells_are_capped():                               # 변경 3,000칸 → len(cells) == 2000
```

- [ ] **Step 2: 실패 확인** — `nice -n 19 python3 -m pytest -q test_map_fix_session.py` → `ModuleNotFoundError`
- [ ] **Step 3: 구현.**
- [ ] **Step 4: 통과 확인** — 같은 명령 → 전부 PASS
- [ ] **Step 5: Commit** — `feat(ros2): map_fix_session — 부분 스캔 진행 단계`

### Task 6 `[잠금: 0단계 ①은 로봇 그래프에 구독자를 붙인다]`: 0단계 실측 — 도킹 스테이션을 지도에 추가

**Files:**
- Create(저장소 밖): `~/vica_data/map_fix_tests/2026-11-19_dock/record_scan.py`, `offline_fix.py`, `README.md`

- [ ] **Step 1: `record_scan.py` 를 쓴다.** rclpy 노드 하나가 `/scan`(sensor_data QoS)을 `--seconds`(기본 8) 동안 받고, TF
  `map→base_footprint`·`base_footprint→<scan frame>` 을 한 번씩 조회해 `scan_<시각>.npz`(ranges 2차원, angle_min,
  angle_increment, pose, sensor, stamp)로 저장한다. 발행·서비스 호출은 하지 않는다.
- [ ] **Step 2: `offline_fix.py` 를 쓴다.** `--map-yaml`(운영 지도의 **사본** 경로)·`--npz`·`--radius` 를 받아 Task 3·4 의
  함수로 자세 다시 맞춤 20회 흩어짐(첫 20장), 변경된 칸, `score_before/after` 를 내고, 수정 전·후 그림(png)과
  `result.json` 을 같은 폴더에 쓴다. 운영 `maps/` 에는 쓰지 않는다.
- [ ] **Step 3 `[잠금]`: 사용자에게 확인받고** 로봇이 도크 근처(도크가 2 m 안)에 서 있을 때
  `nice -n 19 python3 record_scan.py --seconds 8` 을 한 번 돌린다. 벽만 있는 자리에서 한 번 더.
- [ ] **Step 4: `offline_fix.py` 를 두 기록에 돌려** 설계서 7절 ①~⑤ 를 README 에 숫자로 적는다. 목표를 못 넘는 값이
  있으면 Task 3·4 의 상수를 고치고 그 근거를 README 와 코드 주석에 남긴다.
- [ ] **Step 5 `[잠금]`: 실제 지도 반영**(도크를 운영 지도에 넣기)은 Task 9 까지 끝난 뒤 앱 흐름으로 한다 — 도킹 시험이
  멈춘 때 사용자 확인을 받는다.

### Task 7 `[잠금: colcon 빌드]`: 공용 계약 `MapFix.srv`

**Files:**
- Create: `~/wt-map-fix/src/vica_interfaces/srv/MapFix.srv`
- Modify: `~/wt-map-fix/src/vica_interfaces/CMakeLists.txt`(srv 목록 끝 `"srv/GetPatrolRoute.srv"` 다음 줄)

- [ ] **Step 1: `MapFix.srv` 를 쓴다** — 주석은 `SaveKeepout.srv` 문체로, 내용은 아래 그대로.

```text
string map_id
string action      # get | erase_count | erase | scan_check | scan_start | scan_cancel | scan_apply | scan_discard | undo
string args_json
---
bool accepted
string reason
string message
string result_json
```

- [ ] **Step 2: `CMakeLists.txt` 에 `"srv/MapFix.srv"` 한 줄을 더한다.**
- [ ] **Step 3: Commit** — `git -C ~/wt-map-fix commit -m "feat(interfaces): MapFix.srv — 앱 지도 수정 요청(지우기·부분 스캔·되돌리기)"`
- [ ] **Step 4 `[잠금]`: 빌드** — 사용자 확인 뒤 `cd ~/wt-map-fix && nice -n 19 colcon build --packages-select vica_interfaces`
  → `Summary: 1 package finished`. 그 전까지 Task 8 의 노드 시험은 가짜 요청 객체로만 돈다.

### Task 8: `map_fix_node` — 요청 처리·잠금·반영

**Files:**
- Create: `~/wt-map-fix-app/ros2/map_fix_node.py`
- Test: `~/wt-map-fix-app/ros2/test_map_fix_node.py`(`test_keepout_map_node.py` 처럼 `SimpleNamespace` 가짜 노드에
  메서드를 직접 부른다. `pytest.importorskip("rclpy")`.)
- Modify: `~/wt-map-fix-app/ros2/supervisor_bringup.launch.py`(`KEEPOUT_NODE` 옆에 `MAP_FIX_NODE`, 같은 모양의
  `ExecuteProcess`)

**Interfaces:**
- Consumes: Task 1·2 의 `map_fix_store`, Task 5 의 `ScanSession`, `keepout_mask.hold_apply`, `vica_interfaces.srv.MapFix`
  (메서드 안에서 늦게 import — `keepout_map_node._setup_services` 방식), `nav2_msgs.srv.LoadMap`.
- Produces:
```python
IDLE_DIALOG_STATES = frozenset({"", "idle", "none", "arrived", "failed", "estopped"})

def lock_reason(status: str, current_goal: str, dialog_state: str, moving: bool) -> str   # "" 또는 "busy_driving"
def mission_running(dialog_state: str) -> bool                                             # dialog_state.strip() != ""

class MapFixNode(Node):
    # 파라미터: maps_root(기본 keepout_map_node 와 같은 식) · map_load_service("/map_server/load_map") ·
    #           apply_timeout_sec(10.0) · robot_status_topic("/robot_status") · odom_topic("/odom") ·
    #           scan_topic("/scan") · base_frame("base_footprint") · map_frame("map")
    def handle_request(self, request, response)      # 서비스 /vica/map_fix
    def _apply(self, map_id: str) -> tuple[bool, str, str]   # (applied, reason, message) — 설계서 6.3 의 세 갈래
```
- `handle_request` 는 한 번에 하나만 처리한다(`threading.Lock`). 모든 action 은 `MapFixError` 를 잡아
  `accepted=False, reason, message` 로 답한다. 알 수 없는 action 은 `bad_action`.
- action 별 결과(`result_json`)는 설계서 6.2 표 그대로. `erase`·`scan_apply`·`undo` 가 성공하면
  `mission_stale = mission_running(dialog_state)` 를 싣는다.
- `erase`·`undo`·`scan_apply` 의 순서: 잠금 검사 → 백업(되돌리기는 생략) → 파일 쓰기 → `_apply` →
  `apply_failed` 면 `restore_backup` 뒤 `drop_backup` → 응답. `no_nav2`·`apply_timeout`·`not_current_map` 은
  `accepted=True, applied=False`. `apply_timeout` 이면 `_pending_apply = map_id` 로 두고 5 s 타이머가 잠금이 풀린
  때 다시 건다.
- 스캔 세션: `scan_check` 는 스캔 1장으로 `refine_pose` 만(세션을 만들지 않음). `scan_start` 는 이미 세션이 있으면
  `busy_session`. 세션이 있는 동안만 `/scan`·`/odom` 을 구독하고 2 Hz 타이머로 `tick` + `/vica/map_fix/state` 발행.
  세션이 `done`·`failed` 가 되면 마지막 상태를 한 번 더 내고 구독을 푼다.
- 자세 출발값은 TF `map_frame → base_frame`, 라이다 위치는 TF `base_frame → <scan frame>`
  (`pose_bootstrap_node._sensor_offset` 과 같은 조회). 못 얻으면 `no_pose`.

- [ ] **Step 1: 시험을 먼저 쓴다**

```python
def test_lock_reason_table():
    # ("waiting","","idle",False) → "" ; current_goal 있음 → busy_driving ; status "moving" → busy_driving ;
    # dialog_state "waiting"·"seeking"·"navigating"·"paused" → busy_driving ; moving True → busy_driving ;
    # dialog_state "" · "estopped" · "arrived" · "failed" → ""
def test_get_returns_lock_and_history(tmp_path):
def test_erase_count_does_not_touch_files(tmp_path):
def test_erase_on_current_map_backs_up_writes_and_applies(tmp_path):
    # 가짜 _apply 가 (True,"","…") → accepted, applied True, pgm 이 바뀜, history 1줄, mission_stale 은 dialog_state 에 따름
def test_erase_on_other_map_saves_only(tmp_path):            # applied False, reason "not_current_map", accepted True
def test_erase_when_nav2_is_off_saves_only(tmp_path):        # _apply → (False,"no_nav2","Nav2가 실행되면 적용됩니다.")
def test_apply_failed_rolls_the_file_back(tmp_path):         # pgm 바이트가 처음과 같고 history 0줄, accepted False
def test_apply_timeout_keeps_file_and_sets_pending(tmp_path):
def test_locked_requests_are_refused(tmp_path):              # erase·undo·scan_start → busy_driving, 파일 그대로
def test_undo_restores_and_reapplies(tmp_path):
def test_scan_start_needs_current_map_and_nav2(tmp_path):    # not_current_map / no_pose
def test_second_scan_start_is_busy_session(tmp_path):        # Review Focus 4
def test_scan_apply_writes_changes_and_records_history(tmp_path):
    # preview 상태의 가짜 세션(changes 2칸) → pgm 의 그 칸이 0/254, history kind "scan", radius_m, added, removed
def test_unknown_action_and_bad_json(tmp_path):              # bad_action / bad_args
def test_mission_stale_flag(tmp_path):                       # dialog_state "idle" → True, "" → False
```

- [ ] **Step 2: 실패 확인** — `nice -n 19 python3 -m pytest -q test_map_fix_node.py` → `ModuleNotFoundError`
- [ ] **Step 3: 구현.** `main()` 은 `keepout_map_node.main` 과 같은 `MultiThreadedExecutor` 틀. 서비스·타이머는
  `ReentrantCallbackGroup`. `_apply` 는 `keepout_map_node._apply` 의 기다리는 방식(`add_done_callback` + `Event.wait`)을
  쓰되 응답의 `result` 가 `LoadMap.Response.RESULT_SUCCESS` 가 아니면 `apply_failed`.
- [ ] **Step 4: 통과 확인** — 같은 명령 → 전부 PASS(rclpy 가 없으면 SKIP 이 아니라 여기서는 있어야 한다)
- [ ] **Step 5: launch 에 노드를 더하고 Commit** — `feat(ros2): map_fix_node — 지우기·부분 스캔·되돌리기 요청 처리`

### Task 9: 따라 고치는 곳

**Files:**
- Modify: `~/wt-map-fix-app/ros2/map_list_node.py`(`build_map_list` 의 항목 dict, `handle_delete_map`) ·
  Test: `~/wt-map-fix-app/ros2/test_map_list_node.py`
- Modify: `~/wt-map-fix/src/vica_nav2/vica_nav2/pose_bootstrap_node.py`(`_on_map`) ·
  Test(새): `~/wt-map-fix/src/vica_nav2/test/test_pose_bootstrap_map_key.py`

**Interfaces:**
- Produces: `/map_list` 항목에 `"image_version": int`(png 의 `st_mtime_ns // 1_000_000`, 못 읽으면 0).
  `map_list_node.fix_backup_dir(maps_root, map_id) -> Path`. `pose_bootstrap_node.map_key(info, data) -> tuple`
  (기존 여섯 값 + `zlib.crc32(bytes of data)`).

- [ ] **Step 1: 시험을 먼저 쓴다**

```python
# test_map_list_node.py
def test_each_map_carries_an_image_version(tmp_path):        # png 를 다시 쓰면 image_version 이 달라진다
def test_deleting_a_map_also_deletes_its_fix_backups(tmp_path):   # maps/.fix_backup/<id>/ 가 사라지고 다른 지도 것은 남는다
# test_pose_bootstrap_map_key.py  (pytest.importorskip("rclpy"))
def test_same_size_but_different_cells_gives_a_different_key():
def test_identical_map_gives_the_same_key():
```

- [ ] **Step 2: 실패 확인** — `nice -n 19 python3 -m pytest -q test_map_list_node.py -k "image_version or fix_backups"` /
  `cd ~/wt-map-fix && PYTHONPATH=src/vica_nav2 nice -n 19 python3 -m pytest -q src/vica_nav2/test/test_pose_bootstrap_map_key.py` → FAIL
- [ ] **Step 3: 구현.** `_on_map` 은 `key = map_key(info, msg.data)` 한 줄만 바뀐다.
- [ ] **Step 4: 통과 확인** — 위 두 명령 + 기존 `test_map_list_node.py`·`test_pose_score.py` 전부 PASS
- [ ] **Step 5: Commit** — 저장소마다 한 번씩(`feat(ros2): 지도 목록에 그림 버전·지도 삭제 때 수정 백업도` /
  `fix(nav2): pose_bootstrap — 지도 내용이 바뀌면 거리표를 다시 만든다`)

---

## 앱(Flutter) — `~/wt-map-fix-app/lib`

앱 시험·분석 명령(`flutter test`·`flutter analyze`·`dart format`)은 무거워 전부 `[잠금]` 이다. 잠금이 풀리기 전에는
코드와 시험 파일을 쓰기만 하고, 풀린 뒤 Task 14 에서 한꺼번에 돌린다.

| 파일 | 책임 |
|---|---|
| `lib/models/map_fix.dart`(새) | 응답·기록·스캔 상태 모델 |
| `lib/providers/supervisor_provider.dart`(수정) | 서비스 호출·토픽 구독·상태 보관 |
| `lib/models/vica_map.dart`·`lib/widgets/map_canvas.dart`(수정) | 그림 버전으로 옛 그림 안 쓰기 |
| `lib/widgets/map_fix_card.dart`(새) | 지도 설정의 `지도 수정` 칸(75·82) |
| `lib/screens/map_erase_screen.dart`(새) | `지도 부분 지우기`(76·77) |
| `lib/screens/map_rescan_screen.dart`(새) | `지도 다시 스캔하기`(78~81) |
| `lib/screens/save_location_screen.dart`(수정) | 칸 끼워 넣기 |

### Task 10: 모델과 provider

**Files:**
- Create: `lib/models/map_fix.dart` · Test: `test/models/map_fix_test.dart`
- Modify: `lib/providers/supervisor_provider.dart` · Test: `test/providers/map_fix_provider_test.dart`
- Modify: `lib/models/vica_map.dart`, `lib/widgets/map_canvas.dart`(`_imageUrl`, `Image.network` 의 `key`)

**Interfaces:**
- Consumes: `RosBridgeClient.callService({required service, type, args, timeout}) -> RosServiceResponse`,
  `_subscribeRequiredTopics(settings)` 의 `..subscribe(topic:, handler:)`, 시험의 `FakeRosServer`/`server.last.answer(...)`.
- Produces:
```dart
// lib/models/map_fix.dart
class MapFixResult { final bool accepted; final String reason; final String message; final Map<String, Object?> result;
  factory MapFixResult.fromValues(Map<String, Object?> values);   // result_json 을 풀어 result 에. 깨진 JSON 은 {}
  bool get applied; bool get missionStale; }
class MapFixHistoryEntry { final String at; final String kind; final double? radiusM; final int added, removed, black, gray, rectCount;
  factory MapFixHistoryEntry.fromJson(Map<String, Object?> json); }
class MapFixStatus { final String lock; final bool canScan; final List<MapFixHistoryEntry> history;
  factory MapFixStatus.fromResult(Map<String, Object?> result); }
class MapFixEraseCount { final List<({double wM, double hM, int black, int gray})> rects; final int black, gray; factory …fromResult(…); }
class MapFixCell { final double x, y; final bool added; }
class MapFixScanState { final String session, phase, reason; final int remainingS; final double radiusM, movedM, movedDeg,
  scoreBefore, scoreAfter; final int added, removed; final List<MapFixCell> cells; final List<String> ambiguous;
  factory MapFixScanState.fromJson(Map<String, Object?> json); }

// SupervisorProvider
static const mapFixService = '/vica/map_fix';
static const mapFixStateTopic = '/vica/map_fix/state';
Future<MapFixResult> mapFix(String mapId, String action, {Map<String, Object?> args = const {}, Duration? timeout});
  // type 'vica_interfaces/srv/MapFix'. 시간 제한: erase·undo·scan_apply 15 s, 나머지 8 s.
  // 연결이 없거나 예외면 accepted false, reason 'offline', message 에 사유.
MapFixStatus? get mapFixStatus;                  // 마지막 get 결과
Future<void> refreshMapFix(String mapId);        // action 'get'
MapFixScanState? get mapFixScan;                 // 토픽의 최신 상태. 끊기면 null(_clearRobotRuntimeState)
void handleMapFixStateForTest(Map<String, Object?> message);
```
- `VicaMap.imageVersion`(`image_version`, 기본 0). `_imageUrl` 은 0 보다 크면 `?v=<imageVersion>` 을 붙이고,
  `Image.network` 의 key 는 `ValueKey('${map.mapId}:${map.imageVersion}')`.

- [ ] **Step 1: 시험을 먼저 쓴다**

```dart
// test/models/map_fix_test.dart
test('result_json 이 깨져도 죽지 않는다', …);                 // result == {}
test('applied·mission_stale 을 읽는다', …);
test('scan state 의 cells 를 [[x,y,1|-1]] 에서 읽는다', …);
// test/providers/map_fix_provider_test.dart (fakeAsync + FakeRosServer)
test('mapFix 가 map_id·action·args_json 을 보낸다', …);        // server.last.sent 의 args 확인
test('연결이 없으면 offline 으로 답한다', …);
test('토픽 상태가 mapFixScan 에 들어오고 끊기면 지워진다', …);
test('그림 버전이 바뀌면 지도 그림 주소가 바뀐다', …);
```

- [ ] **Step 2: 구현.** 모델은 `pose_check_result.dart` 문체(`fromValues`, `lib/core/json_read.dart` 의 느슨한 읽기).
- [ ] **Step 3: Commit** — `feat(app): 지도 수정 — 모델·서비스 호출·상태 토픽, 지도 그림 버전`

### Task 11: `지도 수정` 칸(75·82)

**Files:**
- Create: `lib/widgets/map_fix_card.dart` · Test: `test/widgets/map_fix_card_test.dart`
- Modify: `lib/screens/save_location_screen.dart`(`_SettingsPanel` 에 `mapFix`, '레일' 패널과 '지도 관리' 패널 사이에
  `VicaExpandPanel(title: '지도 수정', …)`, `_openPanel` 의 미저장 확인은 그대로)

**Interfaces:**
- Produces:
```dart
class MapFixCard extends StatelessWidget {
  const MapFixCard({super.key, required this.connected, required this.status, required this.isDriveMap,
    required this.onErase, required this.onScan, required this.onUndo});
  final bool connected; final MapFixStatus? status; final bool isDriveMap;
  final VoidCallback? onErase, onScan, onUndo;
}
String mapFixStateWord({required bool connected, required MapFixStatus? status});   // '오프라인' | '주행 중' | '가능'
String mapFixDoneMessage(MapFixResult r, {double? before, double? after});
```
- `mapFixDoneMessage`: `applied` 면 `지도를 수정했습니다.`(부분 스캔이면 뒤에 ` 위치 맞춤 62 % → 91 %`), 아니면 로봇의
  `message`. `missionStale` 이면 줄을 바꿔 `Mission Manager에는 적용되지 않았습니다.` / `주행을 껐다 켜면 적용됩니다.`(두 줄)
- 설명 줄 `선택한 곳을 지우거나 다시 스캔합니다`. 버튼 `지우기`·`부분 스캔`(둘 다 `PressScale`). 잠겼으면 둘 다 꺼짐.
  `isDriveMap` 이 거짓이면 `부분 스캔` 만 꺼지고 `현재 지도가 아니면 부분 지우기만 가능합니다.`(11-19 목업 94).
- `최근 수정`: 기록이 있을 때만. 줄 이름 `부분 스캔 · 2 m` / `지우기 · 3곳`, 아랫줄 `11-18 15:42 · 새로 41칸 · 없어짐 28칸` /
  `… · 검은 칸 19개 · 회색 칸 3개`. `되돌리기` 는 맨 윗줄에만 →
  `showVicaConfirm(title: '마지막 수정을 되돌릴까요?', confirmLabel: '되돌리기', danger: true)` → `undo` → `되돌렸습니다.`
- 화면: 패널을 열 때와 수정이 끝날 때 `refreshMapFix(mapId)`. 주행 지도 판정은
  `supervisor.modeStatus?.driveMapId ?? supervisor.currentMapId` 와 보는 지도 비교.

- [ ] **Step 1: 시험을 먼저 쓴다**

```dart
testWidgets('연결이 없으면 오프라인, 잠기면 주행 중, 아니면 가능', …);
testWidgets('잠기면 두 버튼이 꺼진다', …);
testWidgets('주행 지도가 아니면 부분 스캔만 꺼지고 이유가 보인다', …);
testWidgets('기록이 없으면 최근 수정 줄이 없다', …);
testWidgets('되돌리기는 맨 윗줄에만 있다', …);
test('끝 알림 — 적용됨·저장만 됨·미션 알림 덧붙임', …);        // mapFixDoneMessage 세 갈래의 글자 그대로
```

- [ ] **Step 2: 구현.** · **Step 3: Commit** — `feat(app): 지도 설정에 '지도 수정' 칸 — 상태·최근 수정·되돌리기`

### Task 12: `지도 부분 지우기`(76·77)

**Files:**
- Create: `lib/screens/map_erase_screen.dart` · Test: `test/screens/map_erase_screen_test.dart`

**Interfaces:**
- Consumes: `MapCanvas(keepoutEditMode: true, draftKeepoutZone:, onKeepoutPanStart/Update/End:, overlayBuilder:)` —
  드래그 좌표는 ROS m 로 온다. 확정한 사각형은 `overlayBuilder` 로 그린다(빨간 점선, 번호 동그라미).
- Produces: `class MapEraseScreen extends StatefulWidget { const MapEraseScreen({super.key, required this.map}); }` —
  `Navigator.push(MaterialPageRoute<MapFixResult?>(…))` 로 열고 지웠으면 결과를 들고 닫힌다.
- 동작: 드래그가 끝나면 사각형 하나 추가(한 변 0.05 m 미만은 버림, 10곳이면 더 안 받음) → 전체 목록으로 `erase_count` →
  줄마다 `1.1 × 1.1 m`·`검은 칸 14 · 회색 칸 1`·✕, `합계`. `지우기` 는 사각형이 1개 이상일 때만 →
  `showVicaConfirm(title: '선택한 구역을 지울까요?', body: '실제로 없는 장애물인지 다시 확인하세요.', rows: [구역 n곳, 검은 칸, 회색 칸], confirmLabel: '지우기', danger: true)`
  → `erase`. 안내 줄 `지울 곳을 드래그해서 표시하세요`, 칸 제목 `고른 구역`·`n곳`.

- [ ] **Step 1: 시험을 먼저 쓴다**

```dart
testWidgets('사각형을 더하면 erase_count 를 전체 목록으로 부르고 줄이 생긴다', …);
testWidgets('✕ 를 누르면 그 줄만 빠지고 다시 센다', …);
testWidgets('0.05 m 보다 작은 드래그는 버린다', …);
testWidgets('열 곳에서 더 받지 않는다', …);
testWidgets('확인에서 지우기를 누르면 erase 를 보내고 결과를 들고 닫힌다', …);
testWidgets('거절되면 화면에 남고 로봇 문구를 보인다', …);
```

- [ ] **Step 2: 구현.** · **Step 3: Commit** — `feat(app): 지도 부분 지우기 — 사각형 여러 개를 한 번에`

### Task 13: `지도 다시 스캔하기`(78~81)

**Files:**
- Create: `lib/screens/map_rescan_screen.dart` · Test: `test/screens/map_rescan_screen_test.dart`

**Interfaces:**
- Consumes: `MapCanvas(robot: supervisor.primaryRobot, scanHits:, overlayBuilder:)`, `supervisor.mapFixScan`.
- Produces: `class MapRescanScreen extends StatefulWidget { const MapRescanScreen({super.key, required this.map}); }` —
  적용했으면 `MapFixResult` 를 들고 닫힌다. `String mapFixFailText(String reason, List<String> ambiguous)`.
- 단계별 화면(토픽의 `phase` 가 정본):
  - 세션 없음(78): 로봇 자리에 반경 원(`overlayBuilder`, 반지름 = 반경 ÷ resolution 을 화면 배율로), 범위 `1 m`·`2 m`·`3 m`
    (처음 `2 m`), 들어올 때 `scan_check` 한 번 → `위치 맞춤 84 %`·라이다 점. `스캔 시작` → `scan_start`.
  - `refine`·`step_back`·`scanning`(79): 뒤로 가기 없음, 큰 숫자 `remaining_s`·`초 뒤 스캔`, 세 줄 단계, 스캔 중 제목
    `스캔 중`. `중지` → `scan_cancel`.
  - `preview`(80): `cells` 를 칸 크기 네모로(새로 생김 짙은 색 채움, 없어짐 주황 점선), `새로 생김 n칸`·`없어짐 n칸`,
    `수정 전`/`수정 후` %. `적용` → `scan_apply` → 닫힘. `버리기` → `scan_discard`.
  - `failed`(81): `showVicaNotice(title: '여기서는 다시 스캔할 수 없습니다', body: mapFixFailText(…), rows: [위치 맞춤, 구분되지 않는 방향])` 뒤 세션 없음 화면으로.
- `mapFixFailText`: `ambiguous` → `앞뒤|좌우|방향 위치를 구분할 수 없습니다. 모서리나 문틀이 보이는 곳으로 옮겨 주세요.`
  (forward→앞뒤, lateral→좌우, yaw→방향; 방향은 `방향을 구분할 수 없습니다.`) · `low_match`·`no_pose` →
  `지도와 라이다가 맞지 않습니다. 초기 위치를 다시 잡아 주세요.` · `moved` → `로봇이 움직였습니다. 세운 뒤 다시 눌러 주세요.` ·
  `no_change` → `이 범위에서 변경된 곳을 찾지 못했습니다.` · 그 밖은 로봇 `message`.

- [ ] **Step 1: 시험을 먼저 쓴다**

```dart
testWidgets('들어오면 scan_check 를 부르고 위치 맞춤을 보인다', …);
testWidgets('범위를 3 m 로 바꾸고 시작하면 scan_start 에 radius_m 3 이 간다', …);
testWidgets('step_back 상태가 오면 남은 초와 중지 버튼이 보인다', …);
testWidgets('preview 상태가 오면 칸 수와 수정 전·후 % 가 보인다', …);
testWidgets('적용을 누르면 scan_apply 를 보내고 결과를 들고 닫힌다', …);
testWidgets('failed(ambiguous forward) 면 앞뒤 문구 팝업이 뜬다', …);
test('mapFixFailText 사유별 글자', …);
```

- [ ] **Step 2: 구현.** · **Step 3: Commit** — `feat(app): 지도 다시 스캔하기 — 범위·물러나기·미리보기·적용`

### Task 14 `[잠금]`: 묶어 돌리기와 실기

- [ ] **Step 1(사용자 확인 뒤):** `cd ~/wt-map-fix-app && nice -n 19 flutter analyze` → `No issues found!`,
  `nice -n 19 flutter test test/models/map_fix_test.dart test/providers/map_fix_provider_test.dart test/widgets/map_fix_card_test.dart test/screens/map_erase_screen_test.dart test/screens/map_rescan_screen_test.dart` → `All tests passed!`,
  `dart format --output=none --set-exit-if-changed lib` → 변경 없음. 실패는 그 Task 로 돌아가 고친다.
- [ ] **Step 2(사용자 확인 뒤):** Task 7 Step 4 의 빌드, `nice -n 19 python3 -m pytest -q ros2/test_map_fix_*.py`(rclpy 포함) 전부 PASS.
- [ ] **Step 3(사용자 확인 뒤):** 실기 — 설계서 7절 '실기' 목록 그대로. 운영 반영 방법(작업 폴더 판으로 띄울지, 운영
  브랜치에 합칠지)은 그때 사용자에게 묻는다. 첫 장면은 도킹 스테이션을 `vica_map_0630` 에 추가.

### Task 15: 전체 검사 (11-19 사용자 지시 — 구현을 다 마친 뒤)

사용자: "다 구현하고서도 계획한 설계서와 맞는지, 기능은 제대로 하는지, 기능을 해치지 않는 선에서 가볍고 최소한의
자원으로 구성했는지, 앱 UI 는 틀어지거나 화면에 안 맞는 게 있는지, 줄 바꿈은 띄어쓰기 단위로 잘 되는지, 지도
수정 시 틀어짐은 없는지, 틀어짐 방지하는 기능이 잘 들어가 있는지 등 이외에도 전체적으로 싹 검사해줘."

- [ ] **설계서 대조**: 설계서 2~7절·11절을 한 줄씩 짚어 구현·시험이 있는지 표로 만든다(없는 것·다르게 한 것 표시,
  다르게 한 것은 ledger 의 Ruling 과 짝지음).
- [ ] **기능**: 순수 모듈·노드·앱 시험 전부 통과 `[잠금: flutter·rclpy]`. 실기 — 지우기, 부분 스캔(도킹 스테이션 추가),
  되돌리기, 잠금, Nav2 꺼짐, 다른 지도, 미션 알림 `[잠금]`.
- [ ] **자원**: `map_fix_node` 쉬는 동안 CPU·메모리(`ps`, 10분 평균)와 스캔 세션 중 최고값. 비교값
  `map_list_node.py` 46 MB·`keepout_map_node.py` 60 MB. 쉬는 동안 `/scan`·`/odom`·TF 구독이 없는지
  (`ros2 node info` `[잠금]`), numpy·scipy 가 첫 요청 전에 안 올라와 있는지.
- [ ] **앱 화면**: 폰 390·태블릿 820·웹 1440 폭에서 75~82 화면 캡처 — 넘침(overflow)·잘림·겹침 없음, 목업과 대조
  `[잠금: flutter]`. 기록은 `~/vica_data/app_mockups/` 아래에 남긴다.
- [ ] **줄 바꿈**: 새 글자가 전부 `vicaKeepWords`(문장은 `vicaBreakAtSentences`)를 거치는지 코드로 확인하고, 좁은 폭
  캡처에서 낱말 가운데가 끊기지 않는지 본다.
- [ ] **틀어짐 없음**: 실기에서 수정 전·후 `위치 맞춤 %`, 기존 벽과 새로 그린 칸의 이음매(단차·이중 벽) 확인, 같은
  자리 20회 자세 다시 맞춤 흩어짐(0단계 ①), 수정 뒤 그 구간 주행의 AMCL 보정량 비교 `[잠금]`.
- [ ] **틀어짐 방지 기능이 들어가 있는가**: 자세 다시 맞춤(안 바뀐 벽만)·통과 조건(빔 수·비율·RMS·옮긴 양)·구분
  검사·정지 감시(`moved`)·잠금·물러나기 5 s·0.8 N 규칙·주변 1칸 규칙·끝점 앞 0.10 m·미리보기·백업·되돌리기 —
  항목마다 코드 위치와 그것을 깨면 실패하는 시험 이름을 적는다.
- [ ] **그 밖**: 독립 검토자 1명의 브랜치 전체 검토(executing-plans 의 Final Review), 운영 폴더·다른 세션 파일을
  건드리지 않았는지(`git status` 세 저장소), 문구가 83번 보드와 글자 단위로 같은지, 옛 말('지도 고치기'·'다시
  그리기'·'고침')이 코드에 남지 않았는지.
- [ ] 결과를 한 장으로 사용자에게 보고한다(통과·미통과·못 한 것과 이유).
