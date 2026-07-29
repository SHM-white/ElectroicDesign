# Todo 6 HMI 预览契约

此浏览器参考复现 Arduino 地面站固定的 800x480 风格操作界面。它是确定性的测试
工具，不是飞控 UI，也不是显示/触摸端口的替代品。

## 令牌

- Canvas `#F4F4F4`, surface `#FFFFFF`, subtle `#E8E8E8`, border `#C6C6C6`.
- Masthead `#161616`, primary text `#161616`, secondary `#525252`.
- Interactive `#0F62FE`, pressed `#0353E9`, success `#198038`, warning
  `#F1C21B`, error `#DA1E28`.
- 8 px spacing grid, sharp corners, 800x480 target canvas, 48 px minimum
  touch targets, built-in-style sans text with stable numeric columns.

## 界面

界面是扁平的操作仪表板，包括页眉、链路/状态条、选择区、路线/状态区和页脚。不包含
地图、飞行控件、命令传输、装饰图像、渐变、阴影或嵌套卡片。每个状态都同时使用文字
和颜色，在新鲜度相关时显示时长。CJK 标签保持为简短的有界短语，避免在受限视口中
从短语中间断开。

## 状态

预览暴露 `BOOT_LOCKED`、`PRESTART`、`SELECT_PENDING`、`ARMED_READY`、
`CAR_RUNNING` 和 `FAULT`。只有在 `PRESTART` 中才能选择和确认；进入 `ARMED_READY`
前必须收到权威 ACK；小车启动会锁定任务控件，使地面站保持只读。重启返回
`BOOT_LOCKED`。
