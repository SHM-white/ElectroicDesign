# 地面站 LVGL 设计契约

本文档约束 Waveshare ESP32-S3-Touch-LCD-7 Rev1.2 仅显示地面站 UI。目标是带有
GT911 触摸输入的 800x480 RGB 显示屏。仅实现此处定义的界面、字段、状态和控件。

## 0. 研究记录

- IBM Carbon 和 taste-skill 为精确且高可读的
  浅色/中性视觉语言提供了参考。使用源自 Carbon 的令牌，但不得使用 IBM 商标、
  标志或复制品牌专属 UI。
- UI-UX DB 推荐深色 OLED 风格。由于此集成板卡在户外需要浅色界面和深色文字的
  可读性，因此不采用该建议。
- QGroundControl 仅用于参考以状态工具栏优先的信息层级，不是复制布局、控件或
  功能的来源。
- 未使用 Lazyweb，因为没有预配置的身份验证或图像工具，且这是固定的嵌入式硬件，
  不是 Web UI。
- 由于同样的固定硬件限制，且契约不包含装饰图像，未生成图像。

## 1. 范围和设计意图

- 产品界面：供现场操作员使用的固定操作遥测仪表板。
- 设计参数：`DESIGN_VARIANCE 3`、`MOTION_INTENSITY 2`、
  `VISUAL_DENSITY 7`.
- UI 为只读。触摸只切换屏幕，绝不发送飞行命令。
- 严格包含两个屏幕：`Overview` 和 `Detail`。
- 没有地图、图表、无线电、配置控件、命令传输、解锁、起飞、降落或 RTL 控件。
- 没有渐变、阴影、玻璃效果、装饰图像、嵌套卡片、浮动页面区块，也没有按下状态
  反馈之外的动画。

## 2. 颜色令牌

所有值都是 `lv_color_hex()` 接受的精确六位十六进制值，并由 LVGL 量化为 RGB565
输出。不得引入一次性的颜色值。

| 令牌 | Hex | 用途 |
| --- | --- | --- |
| `gs_color_canvas` | `#F4F4F4` | 根背景和未使用的主体区域 |
| `gs_color_surface` | `#FFFFFF` | 扁平内容界面 |
| `gs_color_surface_subtle` | `#E8E8E8` | 字段分隔和禁用填充 |
| `gs_color_border` | `#C6C6C6` | 分隔线和中性轮廓 |
| `gs_color_masthead` | `#161616` | 全宽顶部页眉 |
| `gs_color_masthead_text` | `#FFFFFF` | 页眉标题和链路文字 |
| `gs_color_text_primary` | `#161616` | 主要标签和值 |
| `gs_color_text_secondary` | `#525252` | 辅助标签 |
| `gs_color_text_muted` | `#6F6F6F` | 未知或不可用值的文字 |
| `gs_color_interactive` | `#0F62FE` | 导航按钮填充、焦点和信息 |
| `gs_color_interactive_pressed` | `#0353E9` | 按下的导航按钮 |
| `gs_color_success` | `#198038` | `OK`、新鲜以及锁定/正常状态强调色 |
| `gs_color_warning` | `#F1C21B` | `STALE` 状态强调色 |
| `gs_color_error` | `#DA1E28` | `LOST` 状态强调色 |
| `gs_color_info` | `#0043CE` | 诊断和中性信息强调色 |

语义状态必须始终以文字写出，并同时用颜色显示：
`OK` 使用 `gs_color_success`，`LOST` 使用 `gs_color_error`，`STALE` 使用
`gs_color_warning`，`UNKNOWN` 使用 `gs_color_text_muted`。绝不能仅通过颜色传达状态。

## 3. 字体排印

只能使用 LVGL 内置字体大小。启用的大小严格为 12、14、
16、20、24、32、40 和 48 px；不依赖外部字体资源。实现可在以后启用时，将这些
大小映射到相应的 LVGL 内置字体配置。

| 令牌 | 大小 | 用途 |
| --- | ---: | --- |
| `gs_type_caption` | 12 px | 紧凑的状态限定文字 |
| `gs_type_small` | 14 px | 次要标签和按钮文字 |
| `gs_type_body` | 16 px | 普通字段标签 |
| `gs_type_value` | 20 px | 状态和诊断值 |
| `gs_type_section` | 24 px | 分区标签 |
| `gs_type_display` | 32 px | Overview 界面的 X/Y 位置值 |
| `gs_type_title` | 40 px | 页眉中的屏幕标题 |
| `gs_type_major` | 48 px | 仅保留给单个主导值 |

正文使用内置默认无衬线样式。数值采用等宽/表格化的排版概念：对齐 X 和 Y 值列，保留
正负号，并为数字预留稳定宽度。
这是布局规则，不要求使用外部等宽字体。

## 4. 布局和几何

- 画布固定为 `800 x 480` px。不得添加响应式断点或滚动。
- 使用 8 px 布局网格。所有区域坐标、间距和控件尺寸都是 8 px 的倍数；默认 1 px
  和聚焦控件轮廓 2 px 是唯一的边框例外。
- 根区域：`x=0 y=0 w=800 h=480`，背景为 `gs_color_canvas`，直角 `0 px`。
- 页眉：`x=0 y=0 w=800 h=64`，背景为 `gs_color_masthead`。屏幕标题为
  `x=24 y=8 w=320 h=48`，左对齐并垂直居中。Overview 界面的链路状态为
  `x=600 y=16 w=176 h=32`，右对齐。
- 内容区域：`x=0 y=64 w=800 h=352`。左右可用内边距为 24 px；内容分区为扁平行，
  以 8 px 间距或单条 `gs_color_border` 分隔线分开。
- 页脚：`x=0 y=416 w=800 h=64`，背景为 `gs_color_surface`，水平内边距为 24 px。
- 任何元素都不得与其他区域重叠。值缩短为指定占位符后，文字只能在其自身声明的
  标签边界内裁剪。

## 5. 屏幕契约

### Overview

概览是初始屏幕，优先显示链路状态、状态信息，然后是权威位置。

- 屏幕根区域：使用共享页眉和页脚几何的完整画布。
- 标题标签：`OVERVIEW`、`x=24 y=8 w=320 h=48`、`gs_type_title`、
  `gs_color_masthead_text`。
- 链路标签：`x=600 y=16 w=176 h=32`、`gs_type_value`，右对齐。渲染
  在收到任何经过验证的 V7 帧之前严格渲染 `LINK UNKNOWN`；`link.fresh` 为 true 时
  渲染 `LINK OK`；超过 0.50 s 链路时限后渲染 `LINK LOST`。
- 状态行：`x=24 y=88 w=752 h=80`。标签为 `x=24 y=88 w=752 h=24`。
  值为 `x=24 y=120 w=752 h=48`、`gs_type_value`。渲染
  缺失时渲染 `STATUS UNKNOWN`，不新鲜时渲染 `STATUS STALE`，新鲜时渲染
  `STATUS OK - MODE <mode> <ARMED|LOCKED>`。mode 和 armed 值仅来自 V7 `0x06`。
- 位置行：`x=24 y=192 w=752 h=144`。标签为 `x=24 y=192 w=752 h=24`。
  值为 `x=24 y=224 w=752 h=56`、`gs_type_display`。渲染
  在首个 `0x08` 之前渲染 `POSITION UNKNOWN`，超过 0.20 s 时渲染 `POSITION STALE`，
  新鲜时渲染 `POSITION OK - X <signed> m Y <signed> m`。
  将权威有符号小端 `0x08` 厘米字段 `x_cm` 和 `y_cm` 转换为米显示，保留两位小数。
  `0x51` 集成坐标绝不能替换这些值。
- 位置状态行：`x=24 y=288 w=752 h=32`、`gs_type_body`，必要时重复文字状态并
  使用对应的语义颜色。
- `DETAIL` 导航按钮：`x=680 y=424 w=96 h=48`，文字为 `DETAIL`。

### Detail

详情屏幕显示新鲜度和隔离诊断，但不改变概览位置的权威性。

- 屏幕根区域：使用共享页眉和页脚几何的完整画布。
- 标题标签：`DETAIL`、`x=24 y=8 w=320 h=48`、`gs_type_title`、
  `gs_color_masthead_text`。
- 位置数据年龄行：`x=24 y=88 w=752 h=72`。标签为 `x=24 y=88 w=752 h=24`。
  值为 `x=24 y=112 w=752 h=48`、`gs_type_value`。在首个 `0x08` 之前渲染
  `POSITION AGE UNKNOWN`，超过 0.20 s 时渲染
  `POSITION AGE STALE`，新鲜时渲染 `POSITION AGE <age_s> s`，保留三位小数。
- 诊断行：`x=24 y=184 w=752 h=88`。标签为 `x=24 y=184 w=752 h=24`。
  值为 `x=24 y=208 w=752 h=64`、`gs_type_value`。无数据时渲染 `0x51 UNKNOWN`，
  否则渲染 `0x51 MODE <mode> STATE <state>`。值仅来自
  隔离的 V7 `0x51` 诊断。
- 质量行：`x=24 y=296 w=752 h=72`。标签为 `x=24 y=296 w=752 h=24`。
  值为 `x=24 y=320 w=752 h=48`、`gs_type_value`。`quality` 缺失时渲染 `QUALITY UNKNOWN`，
  否则渲染 `QUALITY <quality>`；不得推断
  遥测未提供的质量等级或门限。
- Back 按钮：`x=24 y=424 w=96 h=48`，文字为 `BACK`。

## 6. 组件和交互状态

- 使用扁平的 LVGL 对象、标签和按钮。行是区域，不是卡片。
- 唯一的交互元素是 `DETAIL` 和 `BACK`。每个触摸目标至少为 48x48 px，两个页脚
  控件与相邻内容至少间隔 8 px。
- 默认按钮：`gs_color_surface` 填充、1 px `gs_color_interactive` 轮廓、
  `gs_color_interactive` 文字，直角。
- 按下按钮：`gs_color_interactive_pressed` 填充和 `gs_color_masthead_text` 文字；
  仅应用即时按下状态反馈。
- 聚焦按钮：保留默认填充，并在声明边界内添加 2 px `gs_color_interactive` 聚焦轮廓。
- 禁用按钮：`gs_color_surface_subtle` 填充、1 px `gs_color_border` 轮廓、
  `gs_color_text_muted` 文字。正常运行中预计不会禁用导航，但必须确定性渲染。
- 仅在按钮释放时导航：`DETAIL` 从 Overview 界面进入 Detail 界面，`BACK` 从 Detail
  界面返回 Overview 界面。按下、长按、拖动和未知目标均不执行操作。
- 不得添加悬停行为、弹窗、模态对话框、工具提示、菜单或动画。

## 7. 遥测和状态语义

- 链路状态仅使用经过验证的 V7 帧新鲜度。保持新鲜至 0.50 s，之后为 `LOST`；这不
  表示位置或状态同样新鲜。
- 状态仅使用 V7 `0x06`：`mode`、`armed` 及其独立的 0.50 s 新鲜度。不得从 `0x51`
  填充状态。
- 位置仅使用 V7 `0x08`：有符号小端 `x_cm`、`y_cm` 及其独立的 0.20 s 新鲜度。过期
  值不得作为当前值显示。
- 详情诊断仅使用 V7 `0x51`：`mode`、`state` 和可选的 `quality`。此数据包不能更新
  位置值、序列号或新鲜度。
- 缺失数据为 `UNKNOWN`，过期数据按上述规定为 `STALE` 或 `LOST`，存在状态标签时
  当前数据为 `OK`。每个状态都要写成文字，也可以使用其语义颜色。
- UI 层不得自行发明任何字段、门限、单位、命令、无线电或传输行为。

## 8. 无障碍和验证

- 对比度：根据实际界面验证主要文字和值；深色页眉上的文字只能使用定义的白色令牌，
  绝不能仅通过颜色传达状态。
- 触摸：每个导航目标至少为 48x48 px，至少间隔 8 px，并提供可见的聚焦和按下状态。
- 可读性：仅使用定义的内置大小、高对比度文字、稳定的数字列、简短的全大写状态标签，
  以及明确的 `UNKNOWN`、`STALE`、`LOST` 和 `OK` 文案。
- 无硬件限制：主机检查可以验证坐标、令牌使用、状态文字和 LVGL 对象边界，但不能证明
  阳光下可读性、GT911 触摸精度、RGB565 外观或实际观看角度。
- 尚待完成的验证仅限 Waveshare 板上的物理显示、触摸和阳光验证。除这些硬件验证外，
  不推迟任何软件行为或设计功能。
