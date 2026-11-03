# SPEC: Writing math in Markdown so it renders in both GitLab and the VSCode preview

<!-- Audience: an AI agent. Chinese version: MARKDOWN_MATH_GUIDE.md (same content; when you edit one, update the other). -->

## Scope

Follow this spec whenever you write or edit any `.md` file in this repo that contains math. There are exactly two target renderers:

| Renderer | Math engine | Accepted delimiters |
|---|---|---|
| GitLab (web) | KaTeX | `$...$`, `$$...$$`, `` $`...`$ ``, ` ```math ` fence |
| VSCode built-in Markdown preview | KaTeX (markdown-it-katex) | `$...$`, `$$...$$` |

Intersection = **use only `$...$` and `$$...$$`**. Anything else shows up as raw text in at least one of them.

Good example: `reference/CapturePointSteppingDesign.md` (passes `check_md_math.py` with 0 errors, 0 warnings).
Bad example: `reference/第一课教案_欧拉角的导数为什么不是角速度.md` (uses `\(...\)` and `\[...\]` throughout; VSCode does not render them).

---

## Rules

### R1 Inline math: `$...$`

```markdown
The CoM height $h$ sets $\omega=\sqrt{g/h}$.
```

- No whitespace right after the opening `$` or right before the closing `$`. `$ \theta$` and `$\theta $` do not render in VSCode.
- No digit right after the closing `$` (`$x$5` does not render). Put a space or punctuation there.
- Keep inline math short. Anything over ~40 characters, or with nested `\frac` or matrices, goes in a display block.

### R2 Display math: `$$` alone on its own line

```markdown
Paragraph before.

$$
\ddot\theta=\omega^2\theta
$$

Paragraph after.
```

- The opening `$$` and the closing `$$` each sit alone on a line, at column 0 (4 spaces of indent turns it into a code block).
- One blank line before and after the block.
- **No blank lines inside the block.** Some renderers cut the formula at the blank line and show the rest as raw text. For visual spacing use `\\[4pt]` or split into two blocks.
- Do not put display math inside list items, tables, or blockquotes (`>`). Put it at top level and break the list around it.
- Never write `text $$x$$ text` inline.

### R3 Multi-line formulas

Wrap them in an environment:

```markdown
$$
\begin{aligned}
x_{cp} &= x+\frac{\dot x}{\omega} \\
       &= 2Ae^{\omega t}
\end{aligned}
$$
```

| Purpose | Environment |
|---|---|
| Derivation aligned on `=` | `aligned` |
| Centered, unaligned lines | `gathered` |
| Piecewise / simultaneous equations | `cases` |
| Matrices | `bmatrix`, `pmatrix` |

A bare top-level `\\` inside a block (no environment) also renders in KaTeX and the good example uses it; existing occurrences may stay, but prefer an environment in new text.

### R4 Only KaTeX-supported commands

Both renderers are KaTeX, not MathJax, not LaTeX. When unsure, check <https://katex.org/docs/supported.html>.

| Forbidden | Why | Use instead |
|---|---|---|
| `\label` `\ref` `\eqref` | no cross-references | words: "see the formula in §2" |
| `\begin{equation}` / `\begin{align}` without `$$` | no delimiter, not rendered | `$$` + `aligned` |
| `\newcommand` used across blocks | each block is isolated | write it out |
| `\usepackage` `\require{...}` `\DeclareMathOperator` | LaTeX / MathJax only | `\operatorname{name}` |
| `\bm` | not in KaTeX | `\boldsymbol` or `\mathbf` |
| Custom macros (e.g. `\J`) | undefined, rendered as red error | write it out, e.g. `J` or `\mathbf J` |

### R5 ASCII only inside math

- No Chinese text or full-width punctuation (`，` `；` `（）`) inside math. Put explanations in the surrounding prose.
- If math must contain words, use `\text{...}` with English only.
- Write Greek letters as commands (`\theta`), not Unicode (`θ`).

### R6 Characters that clash with Markdown

| Situation | Problem | Write |
|---|---|---|
| Formula with `\|` inside a table cell | read as a column separator, table breaks | absolute value `\lvert x\rvert`, norm `\lVert x\rVert`, "such that" `\mid` |
| Currency sign in prose | text between two `$` becomes math | `\$` or "USD" |
| GitLab eats `_` / `*` in inline math as italics | Markdown parsed before math | rewrite first; only if that fails use `` $`...`$ `` (GitLab/GitHub only; VSCode shows it as code with `$` around it) |
| Math in a heading | anchor link becomes garbage | headings are words only; math goes in the body |

### R7 Forbidden delimiters (raw text in VSCode)

~~~text
\( ... \)            inline, LaTeX/ChatGPT style      → use $...$
\[ ... \]            display, LaTeX/ChatGPT style     → use $$ on own lines
\begin{equation}...  without $$ around it             → use $$ + aligned
```math fence        VSCode preview does not render   → use $$
<sup> <sub> HTML     not math, cannot be copied       → use $x^2$ / $x_i$
formula screenshot   not searchable, not diffable     → use $$
~~~

---

## How programmers write math docs (not a paper, not a lecture script)

Readers of this repo are people who write code, and AIs. Write math docs like this, not like a textbook or a lecture script.

| Do | Don't |
|---|---|
| Map every symbol to a code variable in a table: symbol, code name, unit, source file | Symbols only; reader cannot tell which variable is which |
| End the derivation with a form that can be pasted into code, in a code block: `omega = sqrt(g / h)` | LaTeX only; implementer translates by hand |
| Plug in real numbers once (with units) and name the data source (URDF file, yaml, log) | Pure symbolic derivation with no magnitudes |
| Numbers in a table: `quantity \| value \| source` | Numbers scattered through paragraphs |
| Section headings: number + short noun phrase, e.g. `## 3. horizontal position dynamics` | Headings that are questions or chatty sentences |
| Reference by `§2`, file path, `file.cpp:123` | `\eqref{eq:3}`, "as shown in (3.2) above" |
| One display block = one step or one set of simultaneous equations | A whole page of derivation in one block |
| State conclusions directly; mark the formula to implement with `\boxed{}` | Lecture structure ("note, everyone", "at 0:35 in the video", "whiteboard layout") |
| One language per document (Simplified Chinese or English) | Sentences mixing Chinese and English |

---

## Converting a bad file (mechanical steps)

For a file written with `\(...\)` / `\[...\]`:

1. Replace `\(` and `\)` with `$`; remove spaces just inside the `$`.
2. Replace every `\[` with a `$$` on its own line, same for `\]`; remove leading indentation from `$$` lines.
3. If a display block is inside a list item: move it out to top level with blank lines around it.
4. Delete blank lines inside `$$` blocks.
5. Fix `|` inside math in tables (R6).
6. Replace commands KaTeX does not support (R4).
7. Run `python3 reference/check_md_math.py <file>`; it must print `0 error(s)`.
8. Spot-check in the VSCode preview (`Ctrl+Shift+V`), or tell the user which places to spot-check.

Do not finish with one blind `sed` over the whole file: `\[` can appear inside `\\[4pt]`. Before step 2, run `grep -nF '\\['` to check.

## Checker script

After writing or editing any `.md` with math, you must run:

```bash
python3 reference/check_md_math.py <file.md>
```

- Done only when it prints `0 error(s)`. Fix every `ERROR`; fix `WARN` where possible and explain in your reply any you leave.
- The script is a static check and cannot prove rendering is correct; when delivering, tell the user which complex formulas to spot-check in the VSCode preview (`Ctrl+Shift+V`).

---

## Minimal template

````markdown
# <topic>

## Key numbers

| Quantity | Value | Source |
|---|---|---|
| mass $m$ | 69.0 kg | sum of URDF link masses |
| CoM height $h$ | 0.761 m | computed at the `stand_target.yaml` pose |

## 1. <step name>

The horizontal CoM offset is approximately $x \approx h\theta$. Hence

$$
\begin{aligned}
I\ddot\theta &= mgh\theta \\
\ddot\theta  &= \omega^2\theta, \qquad \omega=\sqrt{g/h}
\end{aligned}
$$

## Implementation

$$
\boxed{x_{cp}=x+\frac{\dot x}{\omega}}
$$

```python
omega = math.sqrt(G / com_height)    # rad/s
x_cp = com_x + com_vx / omega        # m
```
````
