# movecobot web UI — DESIGN.md

Design reference for the control panel served on port 8088
(`static/index.html`, `static/style.css`, `static/app.js`) and the temporary
starting page (`scripts/starting_page.py`). All values below are taken from
`static/style.css`; if the two disagree, the CSS wins and this file must be updated.

## 1. Visual theme and atmosphere

- A dense, dark control panel for a real robot arm, not a marketing page.
- Neutral blue-grey dark surfaces, thin 1px borders, one blue accent.
- Flat: no gradients, no decorative imagery, no hazard stripes, no corner brackets.
  The only shadow is on floating popups.
- All UI text and warnings are in English.
- No external resources (fonts, CDNs): the robot PC may have no internet.
- `color-scheme: dark` on `:root` (plus `<meta name="color-scheme">` and
  `<meta name="theme-color" content="#0f1115">`) so native controls and scrollbars are dark.
- Motion (busy dot pulse, spinner) stops under `prefers-reduced-motion: reduce`.

## 2. Color palette and roles

| Token | Hex | Role |
|---|---|---|
| `--bg` | `#0f1115` | Page background, input fields, log box, loading overlay |
| `--panel` | `#171a21` | Header bar, section cards, popups |
| `--panel-2` | `#1e222b` | Section titles, default buttons, value tiles, pills, move rows |
| `--line` | `#2a2f3a` | All borders and dividers |
| `--fg` | `#e6e9ef` | Primary text |
| `--dim` | `#8b93a3` | Labels, secondary text, notes |
| `--accent` | `#4c9aff` | Primary buttons, hover border, busy dot, spinner |
| `--ok` | `#3ecf8e` | Connected / no fault / idle dot, OK log lines |
| `--warn` | `#f5a623` | Warning dot, warning note border, warning log lines |
| `--err` | `#ff5c5c` | Error dot, danger button outline, error log lines |
| `--stop` | `#d92d20` | "Stop motion" button only |

Fixed colors outside the tokens (keep them in sync if the palette changes):

- Text on primary button: `#04101f`
- Tag chip: background `#2b3550`, text `#9fc2ff`, border `#38456b`
- Warning note text: `#e0bd80`
- Popup shadow: `0 12px 32px rgba(0, 0, 0, .5)`
- `scripts/starting_page.py` repeats `--bg`, `--fg`, `--dim`, `--line`, `--accent`
  as literal hex values.

Rules:
- One accent color. Blue means "primary action" or "busy", nothing else.
- Red `--stop` is reserved for "Stop motion". Other destructive actions use the
  outlined `button.danger` style.
- Status is shown by dot color: `ok`, `warn`, `err`, `busy`; no dot class means
  unknown (`--dim`).

## 3. Typography

- UI font: `system-ui, -apple-system, "Segoe UI", Roboto, sans-serif`, 14px, line-height 1.5.
- Numbers, joint values, inputs, log: `--mono` =
  `ui-monospace, SFMono-Regular, "SF Mono", Menlo, Consolas, monospace`.

| Element | Size | Weight / style |
|---|---|---|
| Header title (`h1`) | 15px | 600, letter-spacing .2px |
| Section title (`section > h2`) | 12px | uppercase, letter-spacing .8px, `--dim` |
| Popup title (`.pop h3`) | 15px | default |
| Buttons, inputs, value (`.kv .v`) | 13px | buttons 400, `.primary` 600 |
| Labels, notes, pills, `.sub`, log | 12px | 400 |
| Tag chip | 11px | 400 |
| Stop motion | 13px | 700, letter-spacing .5px |

## 4. Component styles

- **Header** (`header`): sticky, `--panel` background, bottom border, padding 10px 18px,
  gap 16px, wraps on narrow screens. Order: title, status pills, spacer, "Preview before
  moving" toggle, Speed / Accel dropdown, Stop motion.
- **Pill** (`.pill`): radius 999px, `--panel-2` background, `--line` border, 12px `--dim` text,
  contains a `.dot` (8px circle).
- **Section card** (`section`): `--panel`, 1px `--line` border, radius 10px. Title bar in
  `--panel-2` with bottom border; body padding 14px; 14px between stacked sections.
- **Button** (default): `--panel-2` background, `--fg` text, 1px `--line` border, radius 6px,
  padding 7px 10px. Hover: border turns `--accent`. Disabled: opacity .4.
- **Keyboard focus**: buttons, inputs and selects get a 2px `--accent` outline with 2px
  offset on `:focus-visible` only (not on mouse click).
- **Primary button** (`button.primary`): `--accent` fill, dark text, weight 600.
  At most one per group (Go, Save, Grip, Confirm).
- **Danger button** (`button.danger`): outline only, `--err` border and text.
- **Stop motion** (`#stopmove`): `--stop` fill, white text, no border, radius 8px,
  padding 9px 20px. Hover brightness 1.15, active moves down 1px.
- **Input / select**: full width, `--bg` background, 1px `--line` border, radius 6px,
  padding 6px 8px, monospace 13px.
- **Field label** (`label.field`): 12px `--dim` label above the control, 8px bottom margin.
- **Value tile** (`.kv`): key left (`--dim`, 12px), value right (monospace 13px),
  `--panel-2`, radius 6px. Arranged two per row (`.grid6`).
- **Jog row** (`.jogrow`): 34px axis label + two equal buttons (− / +).
- **Saved move row** (`.move`): `--panel-2`, radius 7px, name 600, meta 12px `--dim`,
  actions pushed right. Long names are truncated with an ellipsis (full name in `title`);
  buttons and tag never shrink.
- **Number fields that feed a motion** (jog steps, absolute joint/TCP targets): an empty or
  invalid value is never sent as 0. The browser's validation bubble appears at the field
  and the log gets an error line. Jog steps must be greater than 0. Absolute target
  fields start empty; fill them with "Current".
- **Panel error** (`#pp-err`, `#s-err`): `.note.warn` directly under the panel's buttons,
  hidden when empty. Errors are shown there as well as in the log, because the log may be
  off-screen.
- **Note** (`.note`): 12px `--dim`, 2px left border in `--line`, no radius.
  `.note.warn` uses `--warn` border and `#e0bd80` text.
- **Log** (`#log`, `role="log"`, `aria-live="polite"`): monospace 12px, `--bg`,
  max-height 260px, scrolls; line classes `.t` (time), `.ok`, `.warn`, `.err`.
- **Focus return**: when a popup closes while focus is inside it, focus goes back to the
  button that opened it.
- **Popup** (`.pop`): width 340px (max viewport − 24px), `--panel`, radius 10px, the only
  shadow. Opens directly under the button that triggered it (`.dd > .pop` right-aligned,
  `.pop.confirm` placed under the pressed button). No centered modal, no backdrop.
- **Loading overlay** (`#overlay`): full-screen `--bg`, 30px spinner with `--accent` top
  border, title, and last startup log lines. Same look as the starting page.
- **Preview toggle** (`label.pill.toggle`, `#pv-mode`): a pill with a checkbox, `--accent`
  border when on. Off after every page load. When on, jog buttons and Go only plan paths
  (`/api/preview`); the robot moves only through "Run this path" and its confirm popup.
- **Path preview panel** (`#pv`): full-width section, first in `main`.
  - Left: 3D view `#pv-view` (440px high, `--bg`, 1px `--line`, radius 6px), then a playback
    row `.pv-controls` (Play/Pause, range scrubber, time in monospace, speed select 1×/2×/5×)
    and a `.note` legend.
  - Right: `#pv-info` (12px `--dim`), option rows, `#pv-err` panel error, Discard +
    "Run this path" (primary).
- **Option row** (`.opt`): same surface as a saved move row. Number (monospace), label (600),
  state (`--ok` "OK" / `--err` "Rejected"), monospace meta line (method, largest joint
  rotation, duration) and, for rejected options, the reason. Selected row: `--accent` border.
  Rejected options can be viewed but not run.
- **3D colors**: current arm `#c9ced6` (35% opacity while a path is shown), selected
  runnable path and its gripper box `--accent`, rejected path `--err`, target TCP dot `--ok`,
  grid `--dim` / `--line`, background `--bg`. Two axis triads keep the three.js defaults
  (X red, Y green, Z blue): the base frame (0.15 m, fixed at the origin) and the TCP frame
  (0.06 m, at the tool, turning with it). Both arms carry the TCP triad, so the ghost shows
  the tool orientation along the path. With `prefers-reduced-motion: reduce` the path does
  not autoplay.
- three.js 0.186.0 (MIT) is stored in `static/lib/three/` and loaded through an import map.
  Never load it from a CDN.

## 5. Layout principles

- Top row: Path preview spans all three columns (`grid-column: 1 / -1`); its inner grid
  is `minmax(0,1.7fr)` / `minmax(280px,1fr)` and stacks below 1100px.
- `main` is a 3-column grid: `minmax(320px,1fr)` / `minmax(340px,1.2fr)` / `minmax(300px,1fr)`,
  gap and padding 14px, columns aligned to top.
  - Left: Status, Gripper.
  - Middle: Joint jog, Base jog, Tool jog, Absolute TCP pose, Absolute joint target.
  - Right: Pick and place recording, Saved moves, Log.
- Spacing steps used: 6, 8, 10, 14px. Radius steps: 6px (controls), 7px (rows),
  8px (stop), 10px (cards, popups), 999px (pills).
- Two- and three-column control rows: `.row` (2 equal), `.row3` (3 equal), gap 8px.
- Every panel that moves the robot keeps a short `.note` explaining the frame or effect.

## 6. Depth and elevation

- Level 0: page `--bg`.
- Level 1: header and sections `--panel`.
- Level 2: tiles, buttons, titles `--panel-2`.
- Floating: `.pop` (z-index 40) with shadow; header z-index 20; overlay z-index 60.
- Depth comes from background steps and borders, not shadows.

## 7. Do's and don'ts

Do:
- Use the tokens; add a new token instead of a new literal hex.
- Keep one primary button per group and one accent color on the page.
- Keep robot-moving actions explicit (button press, confirm popup under the button).
- Remove fields or buttons that no longer do anything.
- Show numbers in monospace.

Don't:
- Load web fonts or anything from the internet.
- Use the default "AI look" extras: glowing gradients, glassmorphism, large rounded
  hero cards.
- Add decorative industrial styling (hazard stripes, corner brackets).
- Use red fills except for Stop motion.
- Present "Stop motion" as an emergency stop: it stops the MoveIt execution only;
  the physical e-stop is the safety device.
- Put a fault-clear button in the UI (faults are cleared in the robot's own WebApp).

## 8. Responsive behavior

- Below 1100px width the grid becomes a single column (sections stack in DOM order:
  left, middle, right).
- Header items wrap.
- Popups are limited to `calc(100vw - 24px)`.
- No mobile-specific layout; the panel is meant for a desktop/laptop next to the robot.

## 9. Agent prompt guide

When changing this UI:
- Read `static/style.css` first and reuse the existing classes (`button.primary`,
  `.row`, `.row3`, `label.field`, `.note`, `.kv`, `.pop`).
- Write all visible text in English, short, verb first ("Save pick pose", "Go").
- New robot actions go through the existing confirm popup and the backend API;
  never add auto-retry or automatic recovery in the UI.
- If a color or size changes in `style.css`, update this file and the literal values in
  `scripts/starting_page.py`.
