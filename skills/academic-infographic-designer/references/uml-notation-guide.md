# UML 2.5 Notation Reference

Reference for verifying UML diagram correctness. Use this to decide **what symbols and relationships** to describe in each image-generation prompt. The image model renders the picture; this guide ensures the described content follows standard UML 2.5 so the generated diagram is technically valid.

---

## 1. Class Diagram (Logical View)

### Class Box Structure

Three compartments:
1. **Name** (bold, centered) — preceded by «stereotype» if applicable
2. **Attributes** — visibility prefix + name + `: Type`
3. **Methods** — visibility prefix + name + `(`params`)` + `: ReturnType`

Visibility prefixes: `+` public, `-` private, `#` protected, `~` package. Underline = static. `/` prefix = derived.

### Relationship Notation (describe in prompt)

| Relationship | Visual | Prompt Description |
|---|---|---|
| Association | `A ──── B` | "solid line connecting two classes" |
| Directed Association | `A ───▶ B` | "solid line with open arrowhead from A to B" |
| Aggregation | `A ◇─── B` | "hollow diamond at A connecting to B" |
| Composition | `A ◆─── B` | "filled diamond at A connecting to B" |
| Dependency | `A - - -▶ B` | "dashed line with open arrowhead from A to B" |
| Generalization | `A ───▷ B` | "solid line with hollow triangle at parent B" |
| Realization | `A - - -▷ B` | "dashed line with hollow triangle at interface B" |

### Multiplicity (describe at line ends)

`1` exactly one · `0..1` optional · `*` zero-or-more · `1..*` one-or-more · `n..m` range

### Stereotypes (prompt keywords)

`«interface»` · `«abstract»` (name shown italic) · `«enumeration»` · `«entity»` · `«value_object»` · `«aggregate_root»` · `«service»` · `«repository»` · `«controller»` · `«DTO»` · `«exception»`

---

## 2. Sequence Diagram (Process View)

### Participants

Boxes at top with vertical **dashed lifelines** descending. Human actors drawn as stick figures above their box.

### Message Notation (describe arrows)

| Message | Visual | Prompt Description |
|---|---|---|
| Synchronous call | `──▶` filled arrowhead, solid | "solid arrow from caller to callee with method name" |
| Return | `◄──` open arrowhead, dashed | "dashed return arrow with result label" |
| Asynchronous | `──▷` open arrowhead, solid | "open-arrow solid line for event/fire-and-forget" |
| Self-call | loop on same lifeline | "small loop arrow on the same lifeline" |

**Activation bars**: thin rectangles over lifelines showing active execution period.

**Combined fragments** (describe as labeled boxes over the relevant lifelines):
`alt` (if/else with `[guard]` labels) · `opt` (optional) · `loop` (iteration) · `par` (parallel) · `break` (early exit) · `ref` (references another diagram).

---

## 3. Activity Diagram (Process View)

### Node Types

| Node | Visual | Prompt Description |
|---|---|---|
| Initial | ● filled circle | "filled start circle" |
| Final | ⊙ circle with dot | "bullseye end symbol" |
| Action | rounded rect | "rounded-rectangle action step" |
| Decision | ◇ diamond | "diamond branch with [guard] labels on each side" |
| Fork/Join | ▬ thick bar | "thick horizontal bar for parallel split/sync" |
| Object node | dog-eared rect | "rectangle with folded corner for data" |
| Signal | pentagon | "pentagon for send/receive signal" |

**Swimlanes**: vertical partitions with role labels at top (e.g., Customer | System | Payment).

**Edge guards**: `[condition]` text on decision branches.

---

## 4. Deployment Diagram (Physical View)

### Node Notation

**3D box** for hardware/execution environments (server, container, cloud). Add `«stereotype»` label: `«AWS EC2»`, `«Kubernetes Pod»`, `«RDS»`, `«Docker»`.

**Artifacts** nested inside nodes: small rectangles labeled `«artifact» app.jar` or `«artifact» nginx.conf`.

### Communication Paths (describe labels)

`<<HTTPS>>` · `<<gRPC>>` · `<<AMQP>>` · `<<JDBC>>` · `<<VPN Tunnel>>` — shown as labels on connecting lines. Firewalls drawn as a small rectangle on the line.

**Zone boundaries**: large dashed rectangles grouping nodes by region/VPC, labeled (e.g., "AWS us-east-1", "On-Prem DMZ").

---

## 5. Component Diagram (Development View)

### Component Box

Rectangle with `«component»` stereotype and small component icon on the right edge. Name centered.

### Interface Notation

- **Provided (lollipop)**: circle `○─` on component edge, labeled with interface name.
- **Required (socket)**: half-circle `⊂─` opening toward component, labeled.
- **Dependency**: dashed arrow between components.
- **Delegation**: arrow from external interface to internal part's interface.

---

## 6. Use Case Diagram (Scenarios)

### Elements

- **System boundary**: large dashed rounded rectangle labeled with system name.
- **Actor**: stick figure outside the boundary, name below.
- **Use case**: ellipse inside the boundary, name centered.
- **Association**: solid line actor ↔ use case.
- **«include»**: dashed arrow from base use case to included use case.
- **«extend»**: dashed arrow from extending use case to base use case, optionally with extension-point label.
- **Generalization**: solid line with hollow triangle (actor-to-actor or use case-to-use case).

---

## 7. State Machine Diagram (Logical View)

### Notation

- **State**: rounded rectangle, optional compartments for `entry / do / exit` activities.
- **Initial**: filled circle. **Final**: bullseye.
- **Transition label**: `trigger [guard] / effect`.
- **Choice** ◇ · **Junction** ● · **Fork/Join** ▬ · **Terminate** ✕.

---

## Image Prompt Construction Notes

When describing a diagram to the image model, follow these rules to maximize correctness:

1. **Name the diagram type explicitly** — "UML 2.5 class diagram", "UML 2.5 deployment diagram". This anchors the visual style.
2. **Specify element count up front** — "five class boxes", "three lifelines" — so the model renders the right number of nodes.
3. **Describe relationships in plain words** using the tables above — the model understands "hollow diamond for aggregation" better than raw ASCII.
4. **Keep text labels short** — class names, method names, and stereotypes only. Long descriptions cause garble; expand them in accompanying text instead.
5. **State the layout** — "grid arrangement", "left-to-right flow", "vertical hierarchy with parent on top".
6. **Always append the negative constraints** from SKILL.md (no hand-drawn, no cartoon, no 3D render) so the output stays in clean engineering style.
7. **Split if complex** — if a single diagram needs more than ~12 elements or multiple fragment types, generate two focused images rather than one cluttered one.
