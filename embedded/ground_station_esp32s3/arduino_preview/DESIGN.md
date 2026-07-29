# Todo 6 HMI Preview Contract

This browser reference mirrors the Arduino station's fixed 800x480-style
operator surface. It is a deterministic test harness, not a flight-control
UI and not a replacement for the display/touch port.

## Tokens

- Canvas `#F4F4F4`, surface `#FFFFFF`, subtle `#E8E8E8`, border `#C6C6C6`.
- Masthead `#161616`, primary text `#161616`, secondary `#525252`.
- Interactive `#0F62FE`, pressed `#0353E9`, success `#198038`, warning
  `#F1C21B`, error `#DA1E28`.
- 8 px spacing grid, sharp corners, 800x480 target canvas, 48 px minimum
  touch targets, built-in-style sans text with stable numeric columns.

## Surface

The surface is a flat operational dashboard: masthead, link/status strip,
selection band, route/status band, and footer. No maps, flight controls,
command transport, decorative imagery, gradients, shadows, or nested cards.
Every status uses text as well as color and includes an age where freshness is
relevant. CJK labels are kept in short, bounded phrases so they do not split
mid-phrase on a constrained viewport.

## States

The preview exposes `BOOT_LOCKED`, `PRESTART`, `SELECT_PENDING`, `ARMED_READY`,
`CAR_RUNNING`, and `FAULT`. Selection and confirmation are available only in
`PRESTART`; an authoritative ACK is required before `ARMED_READY`; car start
locks the task controls and leaves the station read-only. Reboot returns to
`BOOT_LOCKED`.
