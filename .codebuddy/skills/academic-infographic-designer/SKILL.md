---
name: academic-infographic-designer
description: Transform any input content into clean scholarly infographics. 支持三种渲染方式：(1) AI 生图（ImageGen）适用于知识卡片/概念总结；(2) SVG 结构化框图（show_widget）适用于技术架构图/系统框图，像素级精确、零重叠；(3) 专利线图。输出为渲染后的图像文件或内联 SVG 矢量图。
agent_created: true
---

# Academic Infographic Designer

## Overview

Transform any input content — concepts, principles, research findings, literature, or systems — into visually clear, scholarly infographics. Output uses a clean academic / scientific illustration style: precise lines, restrained color, crisp typography, generous whitespace. Never hand-drawn, cartoon, or sketch.

**四种渲染方式，按内容类型自动路由**：

| 内容类型 | 渲染方式 | 工具 | 场景说明 |
|---------|---------|------|---------|
| 知识卡片 / 概念图 | AI 生图（学术风格） | ImageGen | 抽象概念、创意可视化 |
| **论文配图 / 科研示意图** | **AI 生图（科研风格）** | **ImageGen** | **学术论文插图、原理说明、收敛色彩** |
| **技术架构图 / 系统框图 / 硬件框图** | **SVG 结构化框图** | **show_widget** | **密集方框精确文字，零重叠零乱码** |
| 专利附图 | AI 生图（黑白线稿） | ImageGen | 单色线图、参考编号标注 |

> 关键原则：**有精确文字和密集方框的架构图走 SVG，有创意性视觉的走 AI 生图。**

---

## 决策路由（CRITICAL — 必须先判断再行动）

接收内容后，按以下流程决定使用哪种渲染方式：

```
接收内容 ──→ 判断内容类型：
              │
              ├─ 架构图 / 系统框图 / 硬件框图
              │  → 路径 B：SVG (show_widget)
              │     （密集方框、精确文字、零重叠）
              │
              ├─ 专利附图（剖面图、装配图）
              │  → 路径 C：AI 生图（黑白线稿风格）
              │     （单色、参考编号标注、≤4 元素）
              │
              ├─ 论文配图 / 科研示意图
              │  → 路径 A：AI 生图（科研风格）
              │     （学术收敛配色、安静白色背景、原理说明）
              │
              └─ 知识卡片 / 概念总结 / 流程图
                 → 路径 A：AI 生图（学术风格）
                    （创意性视觉、抽象概念表达）
```

**强制规则**：画面中 ≥4 个带文字标注的方框 → 强制走路径 B（SVG），不要尝试用 AI 生图画密集方框图。

---

## 路径 A：AI 生图（ImageGen）— 知识卡片 / 概念图 / 科研论文配图

适用于抽象概念、知识总结、创意性可视化。

### Phase 1: Content Analysis & Distillation

1. **Identify the core theme**
2. **Extract 3–7 key points** (odd numbers preferred for balance)
3. **Determine visual representations**
4. **Detect language** — match the input language
5. **Decide layout** — grid / flow / radial / timeline / comparison

### Phase 2: Prompt Construction

Follow `references/style-guide.md` for the prompt template and constraints.

Mandatory negative constraints:
```
no hand-drawn style, no cartoon, no sketch, no doodle, no 3D render,
no photorealistic, no heavy gradient, no drop-shadow gloss, no imperfect lines,
no pencil texture, no watercolor
```

### Phase 3: Image Generation (ImageGen)

- **Size**: Default `1536x1024` (landscape 16:9). Use `1024x1792` for tall content, `1024x1024` for square cards.
- **Quality**: High
- **Style**: Natural rendering

---

## 路径 B（推荐）：SVG 结构化框图（show_widget）— 技术架构图 / 系统框图

适用于软件架构、硬件框图、模块依赖图、UML 图等**有密集方框和精确文字**的场景。

### 为什么用 SVG 而非 AI 生图

| | AI 生图（ImageGen） | SVG（show_widget） |
|---|---|---|
| 方框位置 | 🌫️ 神经网络推测，易重叠 | ✅ 精确 x,y 坐标 |
| 文字渲染 | ❌ 易模糊、错位、乱码 | ✅ 系统字体，完全清晰 |
| 方框数量限制 | ≤4 个，再多就乱 | 6~10 个轻松容下 |
| 修改成本 | 重新生成 | 改几个坐标值 |
| 数据流箭头 | ❌ 经常连错 | ✅ 精确端点控制 |

### 技术规范

参考 Visualizer 的 `read_me`（diagram 模块）获取完整设计系统，关键规则：

#### SVG 画布
- `viewBox="0 0 680 H"`，其中 H 为底部元素 + 20px
- `width="100%"`，背景透明
- 安全区域：x=40 ~ x=640，y=40 ~ y=(H-40)
- 禁止倾斜文字

#### 颜色体系（使用 CSS class）
- c-blue（#378ADD / #85B7EB / #E6F1FB）— 云服务/网络
- c-purple（#7F77DD / #AFA9EC / #EEEDFE）— 固件/应用
- c-teal（#1D9E75 / #5DCAA5 / #E1F5EE）— 硬件/驱动
- c-coral（#D85A30 / #F0997B / #FAECE7）— 告警/监控区域
- c-gray（#888780 / #B4B2A9 / #F1EFE8）— 辅助/标注

#### 方框尺寸
- 单行：`rect_h=44px`，文字居中 `y=起始+22`
- 双行：`rect_h=56px`，标题 `y=起始+38`，副标题 `y=起始+56`
- 宽度公式：`rect_w = max(title_chars × 7, subtitle_chars × 6) + 24`
- 方框间距：最小 60px
- 内边距：24px

#### 箭头标记
```svg
<defs>
  <marker id="arrow" viewBox="0 0 10 10" refX="8" refY="5"
    markerWidth="6" markerHeight="6" orient="auto-start-reverse">
    <path d="M2 1L8 5L2 9" fill="none" stroke="context-stroke"
      stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
  </marker>
</defs>
```

#### 文字
- `.th`（标题）: 500 13px
- `.ts`（副标题）: 400 12px
- `.t`（正文）: 400 14px
- 均使用 `dominant-baseline="central"` 和 `text-anchor="middle"`
- 非中文字体使用 system-ui / sans-serif

#### 术语标注
在图片下方补充术语表：
```markdown
术语表：
1 = ASR 语音识别（Automatic Speech Recognition）
2 = TTS 语音合成（Text-to-Speech）
...
```

### 框图层级规范

```
容器（三层容器结构）
├── rx=20~24，最浅填充（-50 stop），0.5px stroke
├── 标题条：顶部 24px 高，实色填充（-600 stop），白色文字
│
├── 内部模块框（一级）
│   ├── rx=8~12，白色填充 或 浅色填充（-100~200 stop）
│   ├── 标题 13px 500，副标题 12px 400
│   └── 同层高度一致
│
└── 数据流箭头
    ├── 层间：粗实线（1.5px）+ 箭头标记
    └── 同层：细实线（1px）+ 箭头标记
```

### 分层方法

1. 确定逻辑层次数量（典型 2~4 层：云端/固件/硬件）
2. 计算每层可用高度：`layer_h = (total_height - margins) / num_layers`
3. 每个容器由标题条（24px）+ 内部内容区组成
4. 层间用箭头连接，标注数据流方向（如"OPUS Audio / JSON Cmd"）
5. FPS/时序/子模块置于容器底部的小方框区（30px 高）

### 调用方式

先加载 Visualizer design system：
```
read_me(modules: ["diagram"])
```

然后调用 `show_widget` 生成 SVG：
```
show_widget({
  title: "架构图名称（英文连字符）",
  widget_code: `<svg>...</svg>`,
  loading_messages: ["构建容器层", "放置模块组件", "绘制数据流", "最终渲染完成"]
})
```

---

## 路径 C：AI 生图（线稿风格） — 专利附图 / 论文示意图

适用于排版要求简单（≤4 个元素）、对文字精确性要求不高的专利示意图或论文配图。

参考 `references/style-guide.md` 中的 "Patent Figure" 调色板。

---

## Phase 4: Review & Iteration

生成后检查：
1. 是否抓住了核心主题
2. 关键点是否视觉可区分
3. 文字是否可读且语言正确
4. 是否保持了干净专业的风格

如需调整，修改对应路径的参数后重新生成。

---

## 完整用法示例

### Example 1: 知识卡片（→ 路径 A）
**User**: "把『费曼学习法』做成一个知识卡片"
**Action**: ImageGen, 1024x1024, 2x2 grid, Chinese

### Example 2: 架构图（→ 路径 B）
**User**: "画一个 electronBot 系统架构图，三层：云端、固件、硬件"
**Action**: read_me("diagram") → show_widget SVG，精确坐标布局

### Example 3: 微服务架构（→ 路径 B）
**User**: "画一个微服务架构图，含网关、用户服务、订单服务、消息队列"
**Action**: read_me("diagram") → show_widget SVG，5 个方框水平排列 + 箭头

### Example 4: 论文示意图（→ 路径 A）
**User**: "给这篇论文画一张『注意力机制原理』示意图"
**Action**: ImageGen, 1536x1024, Q/K/V attention flow, English labels

### Example 5: 专利附图（→ 路径 C）
**User**: "画一张这个装置的专利附图（剖面）"
**Action**: ImageGen（黑白线稿）, monochrome, reference numerals

---

## References

- `references/style-guide.md` — Scholarly style guide (palettes, layouts, element library, prompt examples)
- `references/4+1-view-guide.md` — 4+1 View Model for architecture diagrams
- `references/uml-notation-guide.md` — UML 2.5 notation reference
