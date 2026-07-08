# ElectronBot-SIM 教学书 HTML 页面模板

> 适用于 `docs/bilibili/` 下所有章节子页面的内容编写规范。
> 每页目标阅读时长 5-12 分钟，由浅入深，逐行递进。

---

## 页面骨架（从上到下）

```
<!DOCTYPE html> + <head> + <style>           ← 固定不变，由 build_book_html.py 生成
<article class="book-page">
  <header class="chap-header">                 ← 标题 + 关键词标签
  <div class="section-background">             ← 本节背景/上下文
  <figure class="diagram">                     ← 架构图（仅章首节）
  <div class="knowledge-card">                 ← 核心概念/本节要点
  <table class="comparison-table">             ← 对比表（有方案对比时）
  <figure class="flow-diagram">                ← 流程图（有流程时）
  <main class="content">                       ← 正文主体
  <div class="glossary-section">              ← 关键词释义表
  <div class="code-index">                     ← 代码位置索引
  <div class="logic-summary">                  ← 实现逻辑小结
  <nav class="chap-nav">                       ← 上一节/目录/下一节
  <footer>
```

---

## 各区块编写规范

### 1. header — 页面标题

```html
<header class="chap-header">
  <div class="chap-kicker">第 02 章 · L1 模型描述层 — 子章节 2/21</div>
  <h1>2.1 确认环境依赖</h1>
  <div class="chap-meta">
    <span>阅读约 5 分钟</span>
    <span>1,314 字</span>
  </div>
  <div class="keywords"><span class="kw-label">核心关键词</span>
    <span class="kw">Python 3.11</span><span class="kw">MuJoCo 3.x</span>...
  </div>
</header>
```

**规则**：
- H1 标题去重：如果正文第一个 H2 与 H1 完全重复（如都写"2.1 xxx"），**删除正文的 H2**，让正文直接从段落开始。
- 关键词标签 8 个以内，只用术语，不用短语。反例：`确认本机 Python 版本` → 正例：`Python 3.11`
- chap-kicker 格式：`第 X 章 · {架构层} — 子章节 {全局序号}/{总节数}`

---

### 2. section-background — 本节背景

```html
<div class="section-background">
  <div class="bg-title">📖 上下文回顾</div>
  <div class="bg-body">
    上一节你装好了 conda 环境……现在你要<b>真正加载 ElectronBot 的 MJCF 模型</b>。
  </div>
</div>
```

**规则**：
- 每页只有一个 background 块（不是两个）。
- 内容承接上一节，引出本节要解决的具体问题。
- 必须包含一个 `<b>` 加粗的核心句。
- 如果该页在章首，额外加一个样式变体（绿色边框）标注架构层定位：
  ```html
  style="background:#f0fdf6;border-left-color:#2f6f4f;"
  ```

---

### 3. knowledge-card — 知识卡片

三种变体：

```html
<!-- 核心概念（绿色边框）：解释一个深层概念 -->
<div class="knowledge-card concept">
  <div class="kc-header">核心概念：XXX</div>
  <div class="kc-body"><p>...</p><ul><li>...</li></ul></div>
  <div class="kc-tags"><span class="kc-tag">XXX</span></div>
</div>

<!-- 本节要点（琥珀色边框）：总结本节产出 -->
<div class="knowledge-card summary">
  <div class="kc-header">本节要点</div>
  ...

<!-- 扩展知识（灰色边框）：补充读物 -->
<div class="knowledge-card extended">
  <div class="kc-header">扩展知识：XXX</div>
  ...
```

**规则**：
- 内容不能复读正文第一句。必须提炼本节的核心概念或产出。
- concept 卡用于解释架构级概念（如 MCP 分界线、运动学链）。
- summary 卡用于实操页（环境搭建、命令执行）。

---

### 4. main.content — 正文主体

**结构层次**：
- `<h2>` = 章节标题（2-3 个）
- `<h3>` = 小节标题（按需，每节 1-2 个）
- `<p>` = 段落。每段 200-400 字，段间有逻辑递进。

**深度标准**：
- 不只说"是什么"，必须解释"为什么"。例如：不只说"Python 3.11 是要求"，要解释"因为 PyTorch/robomimic/SB3 在 3.11 最稳定，3.12/3.13 的 C 扩展未完全适配"。
- 涉及代码时标注文件路径和函数名。格式：`<code>src/electronbot_sim/env.py</code>`
- 涉及固件对标的标注源码位置。格式：`movements.cc:87`
- 变量/参数用 `<code>` 包裹，文件路径用 `<code class="ci-file">`。
- 命令示例必须完整、可复制。使用 `<pre><code class="language-bash">` 包裹。

**代码块格式**：
```html
<pre><code class="language-bash">
conda create -n ebotsim python=3.11 -y
conda activate ebotsim
</code></pre>
```

**表格格式**：使用 `<table class="comparison-table">` 或普通 `<table>`。

---

### 5. glossary-section — 关键词释义表

```html
<div class="glossary-section">
  <div class="gs-title">🔑 关键词释义</div>
  <table class="glossary-table">
    <tr><td>术语</td><td>简短定义（一句话，20-40字）</td></tr>
  </table>
</div>
```

**规则**：
- 至少 3 个术语，最多 6 个。
- 术语优先从 `KEYWORD_DEFS` 全局字典中选取。
- 如果全局字典无对应定义，手动补充。

---

### 6. code-index — 代码位置索引

```html
<div class="code-index">
  <div class="ci-title">📂 本节涉及的代码文件</div>
  <div class="ci-item"><span class="ci-file">assets/mjcf/electronbot.xml</span>
    <span class="ci-desc">— 机器人模型本体（7 body/6 joint/6 actuator）</span></div>
  <div class="ci-item"><span class="ci-file">src/electronbot_sim/env.py</span> ·
    <span class="ci-func">ElectronBotEnv.step()</span>
    <span class="ci-desc">— 执行单步仿真推进</span></div>
</div>
```

**规则**：
- 文件路径用 `<span class="ci-file">`，函数名用 `<span class="ci-func">`，描述用 `<span class="ci-desc">`。
- 4-5 条为宜，不超过 6 条。
- 优先列出本节直接涉及的代码文件，从架构文档中获取精确路径。

---

### 7. logic-summary — 实现逻辑小结

```html
<div class="logic-summary">
  <div class="ls-title">⚙️ 实现逻辑</div>
  <ol>
    <li><b>建模先行</b>：从 STEP 图纸出发...解决"长什么样"的问题。</li>
    <li><b>引擎驱动</b>：MuJoCo 加载 MJCF...</li>
  </ol>
</div>
```

**规则**：
- 不能是 H2 标题列表（反例：`<li>2.1 章节目标</li>`）。
- 必须是逻辑步骤摘要，每步以 `<b>` 加粗关键词开头。
- 4-6 步为宜。
- 章级页面覆盖全链路，节级页面聚焦本节流程。

---

### 8. 图表增强元素

| 类型 | class | 何时用 |
|------|-------|--------|
| 架构图 | `diagram` | 仅章首节，展示该章对应的架构层 |
| 对比表 | `comparison-table` | 两个以上方案/参数/模式对比 |
| 流程图 | `flow-diagram` | 多步骤过程或决策分支 |
| 数据图 | `data-chart` | 展示性能数据或分布 |
| 配图 | `figure-wrap` | 概念或流程的插图 |

---

## 全局约束

1. **不重复**：H1/H2 不重复，background 不重复正文，knowledge-card 不复读首句。
2. **有深度**：每句话要么在解释"是什么"，要么在解释"为什么这样设计"。
3. **可执行**：所有命令完整、直接复制可用。
4. **有来源**：代码路径和函数名来自 `docs/概要设计/` 和 `docs/tasks/` 中的真实文件。
5. **有对标**：仿真行为 1:1 对齐真机固件时，标注固件源码位置（如 `movements.cc:87`）。
6. **渐进式**：每页从上一页的产出出发，引出本节问题，再展开解决。

---

## 页面类型速查

| 页面类型 | background 标题 | knowledge-card 类型 | 图表 | 示例页 |
|----------|-----------------|---------------------|------|--------|
| 章首/架构概述 | 架构定位（绿框） | concept | diagram（必选） | 01-01, 03-01, 04-01 |
| 环境/配置实操 | 上下文回顾 | summary | comparison-table | 02-01, 02-02 |
| 流程/管线详解 | 上下文回顾 | concept | flow-diagram | 03-03, 06-02 |
| 对比/决策 | 上下文回顾 | summary | comparison-table | 03-06, 05-02 |
| 问题排查/校准 | 上下文回顾 | summary | — | 07-02 |

---

```xml
<summary>
  <title>ElectronBot-SIM 教学书 HTML 页面模板</title>
  <description>对整个文档的简介描述</description>
  <tags>HTML模板, 页面规范</tags>
</summary>
```
