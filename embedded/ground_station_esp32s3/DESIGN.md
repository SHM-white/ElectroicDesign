# Ground Station LVGL Design Contract

This document is binding for the Waveshare ESP32-S3-Touch-LCD-7 Rev1.2
display-only ground-station UI. The target is an 800x480 RGB display with
GT911 touch input. Implement only the surfaces, fields, states, and controls
defined here.

## 0. Research Log

- IBM Carbon and the taste-skill informed the precise, high-legibility,
  light/neutral visual language. Use Carbon-derived tokens without IBM
  trademarks, logos, or copied brand-specific UI.
- The UI-UX DB recommended a dark OLED treatment. It was rejected because
  outdoor legibility on this integrated board requires light surfaces and
  dark text.
- QGroundControl informed only the status-toolbar-first information
  hierarchy. It is not a source for copied layouts, controls, or features.
- Lazyweb was skipped because no preconfigured authentication or image tool
  is available, and this is fixed embedded hardware rather than a web UI.
- Image generation was skipped for the same fixed-hardware constraint and
  because the contract has no decorative imagery.

## 1. Scope and Design Intent

- Product surface: a field operator's fixed operational telemetry dashboard.
- Design parameters: `DESIGN_VARIANCE 3`, `MOTION_INTENSITY 2`,
  `VISUAL_DENSITY 7`.
- The UI is read-only. Touch changes screens only; it never sends flight
  commands.
- There are exactly two screens: `Overview` and `Detail`.
- There are no maps, charts, radios, configuration controls, command
  transport, arm, takeoff, land, or RTL controls.
- There are no gradients, shadows, glass effects, decorative imagery, nested
  cards, floating page sections, or animations beyond pressed-state feedback.

## 2. Color Tokens

All values are exact six-digit hex values accepted by `lv_color_hex()` and
quantized by LVGL for RGB565 output. Do not introduce one-off color values.

| Token | Hex | Use |
| --- | --- | --- |
| `gs_color_canvas` | `#F4F4F4` | Root background and unused body area |
| `gs_color_surface` | `#FFFFFF` | Flat content surface |
| `gs_color_surface_subtle` | `#E8E8E8` | Field separation and disabled fill |
| `gs_color_border` | `#C6C6C6` | Dividers and neutral outlines |
| `gs_color_masthead` | `#161616` | Full-width top masthead |
| `gs_color_masthead_text` | `#FFFFFF` | Masthead title and link text |
| `gs_color_text_primary` | `#161616` | Primary labels and values |
| `gs_color_text_secondary` | `#525252` | Supporting labels |
| `gs_color_text_muted` | `#6F6F6F` | Unknown or unavailable value text |
| `gs_color_interactive` | `#0F62FE` | Navigation button fill, focus, and info |
| `gs_color_interactive_pressed` | `#0353E9` | Pressed navigation button |
| `gs_color_success` | `#198038` | `OK`, fresh, and locked/normal state accent |
| `gs_color_warning` | `#F1C21B` | `STALE` state accent |
| `gs_color_error` | `#DA1E28` | `LOST` state accent |
| `gs_color_info` | `#0043CE` | Diagnostic and neutral informational accent |

Semantic state must always be written as text as well as shown with color:
`OK` uses `gs_color_success`, `LOST` uses `gs_color_error`, `STALE` uses
`gs_color_warning`, and `UNKNOWN` uses `gs_color_text_muted`. Color alone is
never a status signal.

## 3. Typography

Use LVGL built-in font sizes only. The enabled size set is exactly 12, 14,
16, 20, 24, 32, 40, and 48 px; there is no external font asset dependency.
The implementation may map these sizes to the matching built-in LVGL font
configuration when enabled later.

| Token | Size | Use |
| --- | ---: | --- |
| `gs_type_caption` | 12 px | Compact status qualifier |
| `gs_type_small` | 14 px | Secondary labels and button text |
| `gs_type_body` | 16 px | Normal field labels |
| `gs_type_value` | 20 px | Status and diagnostic values |
| `gs_type_section` | 24 px | Section labels |
| `gs_type_display` | 32 px | Overview X/Y position value |
| `gs_type_title` | 40 px | Screen title in the masthead |
| `gs_type_major` | 48 px | Reserved for a single dominant value only |

Use the built-in default sans treatment for prose. Numeric values use a
monospaced/tabular conceptual treatment: align the X and Y value columns,
keep signs visible, and reserve stable width for digits. This is a layout
rule, not a request for an external monospaced font.

## 4. Layout and Geometry

- Canvas is fixed at `800 x 480` px. Do not add responsive breakpoints or
  scrolling.
- Use an 8 px layout grid. All region coordinates, gaps, and control sizes
  are multiples of 8 px; the 1 px default and 2 px focused control outlines
  are the only border exceptions.
- Root: `x=0 y=0 w=800 h=480`, background `gs_color_canvas`, sharp `0 px`
  corners.
- Masthead: `x=0 y=0 w=800 h=64`, background `gs_color_masthead`.
  Screen title is `x=24 y=8 w=320 h=48`, left aligned and vertically
  centered. Overview link state is `x=600 y=16 w=176 h=32`, right aligned.
- Content region: `x=0 y=64 w=800 h=352`. Its usable inset is 24 px on the
  left and right; content sections are flat rows separated by 8 px or a
  single `gs_color_border` divider.
- Footer: `x=0 y=416 w=800 h=64`, background `gs_color_surface`, with a
  24 px horizontal inset.
- No element may overlap another region. Text is clipped only within its
  own declared label bounds after the value has been shortened to the
  specified placeholder.

## 5. Screen Contracts

### Overview

The overview is the initial screen and prioritizes link state, status, then
authoritative position.

- Screen root: full canvas with the shared masthead and footer geometry.
- Title label: `OVERVIEW`, `x=24 y=8 w=320 h=48`, `gs_type_title`,
  `gs_color_masthead_text`.
- Link label: `x=600 y=16 w=176 h=32`, `gs_type_value`, right aligned. Render
  exactly `LINK UNKNOWN` before any verified V7 frame, `LINK OK` while
  `link.fresh` is true, or `LINK LOST` after the 0.50 s link limit.
- Status row: `x=24 y=88 w=752 h=80`. Label is `x=24 y=88 w=752 h=24`.
  Value is `x=24 y=120 w=752 h=48`, `gs_type_value`. Render
  `STATUS UNKNOWN` when absent, `STATUS STALE` when not fresh, or
  `STATUS OK - MODE <mode> <ARMED|LOCKED>` when fresh. The mode and armed
  value come only from V7 `0x06`.
- Position row: `x=24 y=192 w=752 h=144`. Label is `x=24 y=192 w=752 h=24`.
  Value is `x=24 y=224 w=752 h=56`, `gs_type_display`. Render
  `POSITION UNKNOWN` before the first `0x08`, `POSITION STALE` when older
  than 0.20 s, or `POSITION OK - X <signed> m Y <signed> m` when fresh.
  Convert the authoritative signed little-endian `0x08` centimeter fields
  `x_cm` and `y_cm` to meters for display, with two decimal places. The
  `0x51` integrated coordinates must never replace these values.
- Position state line: `x=24 y=288 w=752 h=32`, `gs_type_body`, repeats the
  text state when needed and carries the corresponding semantic color.
- Detail button: `x=680 y=424 w=96 h=48`, text `DETAIL`.

### Detail

The detail screen exposes freshness and the isolated diagnostic without
changing the authority of the overview position.

- Screen root: full canvas with the shared masthead and footer geometry.
- Title label: `DETAIL`, `x=24 y=8 w=320 h=48`, `gs_type_title`,
  `gs_color_masthead_text`.
- Position age row: `x=24 y=88 w=752 h=72`. Label is `x=24 y=88 w=752 h=24`.
  Value is `x=24 y=112 w=752 h=48`, `gs_type_value`. Render
  `POSITION AGE UNKNOWN` before the first `0x08`, `POSITION AGE STALE` after
  0.20 s, or `POSITION AGE <age_s> s` while fresh, with three decimal places.
- Diagnostic row: `x=24 y=184 w=752 h=88`. Label is `x=24 y=184 w=752 h=24`.
  Value is `x=24 y=208 w=752 h=64`, `gs_type_value`. Render `0x51 UNKNOWN`
  when absent, otherwise `0x51 MODE <mode> STATE <state>`. Values come only
  from the isolated V7 `0x51` diagnostic.
- Quality row: `x=24 y=296 w=752 h=72`. Label is `x=24 y=296 w=752 h=24`.
  Value is `x=24 y=320 w=752 h=48`, `gs_type_value`. Render `QUALITY UNKNOWN`
  when `quality` is absent, otherwise `QUALITY <quality>`; do not infer a
  quality grade or threshold not supplied by telemetry.
- Back button: `x=24 y=424 w=96 h=48`, text `BACK`.

## 6. Components and Interaction States

- Use flat LVGL objects, labels, and buttons. Rows are regions, not cards.
- The only interactive elements are `DETAIL` and `BACK`. Each touch target is
  at least 48x48 px and the two footer controls have at least 8 px separation
  from adjacent content.
- Default button: `gs_color_surface` fill, 1 px `gs_color_interactive`
  outline, `gs_color_interactive` text, sharp corners.
- Pressed button: `gs_color_interactive_pressed` fill and
  `gs_color_masthead_text` text; apply immediate pressed-state feedback only.
- Focused button: retain the default fill and add a 2 px
  `gs_color_interactive` focus outline inside the declared bounds.
- Disabled button: `gs_color_surface_subtle` fill, 1 px `gs_color_border`
  outline, `gs_color_text_muted` text. Disabled navigation is not expected
  during normal operation but must render deterministically.
- Navigation occurs only on button release: `DETAIL` goes Overview to Detail
  and `BACK` goes Detail to Overview. Press, hold, drag, and unknown targets
  do nothing.
- Do not add hover behavior, popups, modal dialogs, tooltips, menus, or
  animation.

## 7. Telemetry and Status Semantics

- Link state uses verified V7 frame freshness only. It is fresh through 0.50 s
  and then `LOST`; it does not imply that position or status is fresh.
- Status uses V7 `0x06` only: `mode`, `armed`, and its independent 0.50 s
  freshness. It must not be populated from `0x51`.
- Position uses V7 `0x08` only: signed little-endian `x_cm`, `y_cm`, and its
  independent 0.20 s freshness. A stale value is not displayed as current.
- Detail diagnostics use V7 `0x51` only: `mode`, `state`, and optional
  `quality`. This packet cannot update position value, sequence, or freshness.
- Missing data is `UNKNOWN`, expired data is `STALE` or `LOST` as specified
  above, and current data is `OK` where a state label is present. Every
  state is written in text and may also use its semantic color.
- No field, threshold, unit, command, radio, or transport behavior may be
  invented by the UI layer.

## 8. Accessibility and Validation

- Contrast: validate primary text and values against their exact surfaces;
  use dark masthead text on the dark masthead only with the defined white
  token, and never communicate state through color alone.
- Touch: keep every navigation target at least 48x48 px with at least 8 px
  separation, and provide visible focused and pressed states.
- Readability: use only the defined built-in sizes, high-contrast text,
  stable numeric columns, short all-caps state labels, and explicit
  `UNKNOWN`, `STALE`, `LOST`, and `OK` wording.
- No-hardware limitations: host review can check coordinates, token use,
  state text, and LVGL object bounds, but cannot prove sunlight readability,
  GT911 touch accuracy, RGB565 appearance, or physical viewing angles.
- Accepted debt is limited to physical-board display, touch, and sunlight
  validation on the Waveshare board. No software behavior or design feature
  is deferred under this debt.
