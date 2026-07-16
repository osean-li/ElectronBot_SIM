# Academic Infographic Style Guide

This guide defines the clean, scholarly vector / flat / technical aesthetic used by the Academic Infographic Designer skill. It mirrors the structure of the Infographic Maker style guide, but replaces hand-drawn cartoon styling with precise, modern, research-oriented rendering suited to papers and patents.

## Core Style Keywords

Always include these in every prompt to maintain consistency:

```
clean flat vector illustration, precise geometry, crisp edges, flat colors,
minimal subtle shading, geometric icons, modern sans-serif typography,
whitespace-rich layout, professional technical aesthetic,
no hand-drawn elements, no cartoon, no sketch, no 3D render
```

## Color Palette Presets

Professional, restrained palettes. Each has a primary, an accent, and a background. Use 2–3 colors maximum; flat fills, subtle background washes only.

| Theme | Primary | Accent | Background |
|-------|---------|--------|------------|
| Technology | Slate blue `#2563EB` | Cyan `#06B6D4` | White `#FFFFFF` |
| Business | Indigo `#4F46E5` | Amber `#F59E0B` | Off-white `#FAFAFA` |
| Science | Teal `#0D9488` | Violet `#7C3AED` | White `#FFFFFF` |
| Education | Blue `#2563EB` | Emerald `#10B981` | Light gray `#F8FAFC` |
| Health | Sky `#0EA5E9` | Rose `#F43F5E` | White `#FFFFFF` |
| Creative | Violet `#7C3AED` | Pink `#EC4899` | Off-white `#FAF9FB` |
| Finance | Navy `#1E3A8A` | Green `#059669` | White `#FFFFFF` |

### Academic / Scientific / Patent Palettes

For research papers, theses, and patent figures, prefer muted scholarly tones with high print contrast. Use a single accent at most; let line weight and labels carry the structure.

| Style | Primary (lines/labels) | Accent | Background |
|-------|------------------------|--------|------------|
| Classic Paper | Ink `#1A1A1A` | Navy `#1F3A5F` | White `#FFFFFF` |
| Muted Scholar | Slate `#334155` | Burgundy `#7F1D1D` | White `#FFFFFF` |
| STEM Journal | Charcoal `#27272A` | Forest `#166534` | White `#FFFFFF` |
| Patent Figure | Black `#000000` | None (monochrome) | White `#FFFFFF` |
| Blueprint | Navy `#0F2A4A` | Cyan `#1E7296` | Pale blue `#EEF4F8` |

**Patent-figure convention**: monochrome only (black lines on white), no fills or color, reference numerals in circles (e.g., `① ② ③`) instead of descriptive labels. State explicitly in the prompt when the target is a patent drawing.

## Layout Patterns

Use the same vocabulary as Infographic Maker, but render each cleanly.

#### Grid Layout (best for 4–6 equal-weight points)
```
Organize content in a [2x2 / 2x3 / 3x2] grid. Each cell: a clean geometric icon
at the top with a keyword below. Thin divider lines or subtle flat card backgrounds
between cells. Consistent spacing.
```

#### Flow Layout (best for processes or sequences)
```
Content flows left-to-right connected by clean straight or gently curved arrows.
Each step shown as a flat rounded rectangle or circle with an icon and a label.
Consistent horizontal spacing.
```

#### Radial Layout (best for a central concept with related ideas)
```
Central concept in a clean circle at the center. Related ideas radiate outward
connected by straight lines. Each satellite idea has its own small flat icon.
```

#### Timeline Layout (best for chronological content)
```
Horizontal clean line across the center. Events marked with flat markers
(circles/squares) above and below the line. Dates or labels in clean text.
Minimal decoration.
```

#### Comparison Layout (best for vs / pros-cons content)
```
Split layout with a clean vertical divider in the center. Left and right sides use
contrasting accent colors. Matching elements on each side for easy comparison.
Flat cards.
```

## Visual Element Library

Clean, geometric representations (no doodles):

- **People**: Neutral stylized figures — simple geometric bodies, minimal faces, or icon-only representation. Never caricatures.
- **Buildings**: Precise rectangular forms, flat shading, clean window grids.
- **Devices**: Simplified flat shapes, thin screen borders, minimal detail.
- **Nature**: Simple geometric trees/clouds, flat fills.
- **Data**: Clean flat bar charts, pie/donut charts with thin borders, upward arrows.
- **Connections**: Straight or gently curved arrows, thin lines, clean nodes.
- **Emphasis**: Thin underlines, subtle highlight bars, small clean badges, numbered circles.

## Text Rendering Tips

Since AI image generation has limitations with text rendering:

1. **Minimize text in the prompt** — focus on visual elements and use text sparingly.
2. **Use short keywords** (1–3 words per label) rather than phrases or sentences.
3. **Title text** should be 3–6 words maximum.
4. **Prefer icons over labels** where the meaning is obvious.
5. **Number lists** (1, 2, 3) render more reliably than words when sequence matters.

## Quality Boosters

Add these modifiers for higher quality output:

```
professional graphic design quality, clean composition,
balanced visual weight, cohesive color scheme,
editorial infographic style, crisp vector rendering
```

## Negative Guidance

Explicitly exclude these in every prompt:

```
no photography, no 3D rendering, no stock photo style,
no hand-drawn sketch, no cartoon, no doodle, no pencil texture,
no watercolor, no glossy 3D, no overly saturated neon,
no cluttered decoration
```

## Full Prompt Examples

### Example: AI Concepts (Chinese)

```
A clean professional infographic about artificial intelligence core concepts, modern flat vector style, landscape 16:9.

Title: "AI 核心概念" in bold clean sans-serif at the top center.

Main content organized in a 2x3 grid layout:
- Top left: flat robot icon with a glowing brain node representing "机器学习"
- Top center: two connected network nodes representing "深度学习"
- Top right: speech bubble from a flat screen representing "自然语言处理"
- Bottom left: clean eye icon representing "计算机视觉"
- Bottom center: flat robotic arm icon representing "机器人技术"
- Bottom right: clean orbit icon with graph lines representing "预测分析"

Thin divider lines between cells. Small geometric accents (gears, nodes) in the margins.

Style: clean flat vector illustration, precise edges, slate blue and cyan palette on white background, geometric icons, modern sans-serif Chinese text, whitespace-rich layout, no hand-drawn or cartoon elements. Professional graphic design quality, clean composition.
```

### Example: Agile Workflow (English)

```
A clean professional infographic about agile project management workflow, modern flat vector style, landscape 16:9.

Title: "The Agile Journey" in bold clean sans-serif at the top.

Main content organized as a circular flow layout:
- Starting point: a flat list icon labeled "Backlog"
- arrow flows to: a flat team icon labeled "Sprint Planning"
- arrow flows to: a flat calendar icon labeled "Daily Standup"
- arrow flows to: a flat presentation icon labeled "Sprint Review"
- arrow flows to: a flat magnifier icon labeled "Retrospective"
- curved arrow loops back to Sprint Planning

Center of the cycle: clean text "ITERATE" with circular arrows.

Style: clean flat vector illustration, precise edges, indigo and amber palette on off-white background, geometric icons, modern sans-serif English text, no hand-drawn or cartoon elements.
```

### Example: Patent Figure (Monochrome)

```
A precise patent-style technical drawing of a device cross-section, monochrome line art, white background.

Strictly black thin lines on pure white, no color, no shading, no fills.
Components labeled with circled reference numerals: ① main body, ② sensor, ③ connector, ④ housing.
Clean orthogonal lines, precise right angles, technical drafting aesthetic.

Style: patent figure, line drawing, monochrome black on white, precise technical illustration,
no hand-drawn, no cartoon, no sketch, no 3D render, no color.
```
