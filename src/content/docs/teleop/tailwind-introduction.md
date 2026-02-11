---
title: "Tailwind Introduction"
---
# Tailwind Introduction

Tailwind CSS is a utility-first CSS framework. Instead of writing custom CSS rules, you apply small, single-purpose classes directly in your HTML. Click [here](https://tailwindcss.com/docs) to access the Tailwind docs.

## Why Tailwind over Bootstrap?

- **Utility-first, not opinionated**: Tailwind gives you low-level utilities instead of pre-built components. You build your own designs without fighting framework defaults.
- **JIT compiled**: Only the classes you use end up in your CSS bundle. Bootstrap ships its entire pre-compiled stylesheet.
- **Industry standard**: Tailwind is the most widely used CSS framework in modern frontend development.

## When to Use Tailwind vs `<style scoped>`

Since Vue uses SFCs (Single File Components), using `<style scoped>` is already convenient. However, prefer Tailwind utility classes for common styling needs like spacing, layout, and colors.

Use `<style scoped>` only when:
- You need custom styles not covered by Tailwind (e.g., a unique animation)
- You are defining reusable component-level classes (our `cmd-*` design system classes)

## Spacing Utility

Throughout the codebase, you'll see classes like `px-2` and `m-3`. These control padding and margin.

### Property
- `m` - margin
- `p` - padding

### Sides
- `t` - top
- `b` - bottom
- `l` - left
- `r` - right
- `x` - left and right (horizontal)
- `y` - top and bottom (vertical)
- (omitted) - all sides

### Size (spacing scale)

Tailwind uses a consistent spacing scale where each unit = 0.25rem (4px):

| Class | Value |
|-------|-------|
| `0` | 0 |
| `1` | 0.25rem (4px) |
| `2` | 0.5rem (8px) |
| `3` | 0.75rem (12px) |
| `4` | 1rem (16px) |
| `5` | 1.25rem (20px) |
| `6` | 1.5rem (24px) |
| `8` | 2rem (32px) |
| `10` | 2.5rem (40px) |
| `12` | 3rem (48px) |

### Examples

| Class | Description |
|-------|-------------|
| `m-4` | Margin on all sides = 1rem |
| `pt-2` | Padding top = 0.5rem |
| `py-0` | Padding top & bottom = 0 |
| `ml-auto` | Margin left = auto |
| `px-6` | Padding left & right = 1.5rem |

## Layout Utilities

Tailwind's flexbox utilities replace verbose CSS:

| Class | CSS |
|-------|-----|
| `flex` | `display: flex` |
| `flex-col` | `flex-direction: column` |
| `items-center` | `align-items: center` |
| `justify-between` | `justify-content: space-between` |
| `gap-2` | `gap: 0.5rem` |
| `w-full` | `width: 100%` |
| `h-full` | `height: 100%` |
| `grow` | `flex-grow: 1` |

### Example

```html
<div class="flex flex-col gap-2 h-full">
  <div class="flex justify-between items-center">
    <h4>Title</h4>
    <span>Status</span>
  </div>
  <div class="grow">Content</div>
</div>
```

## Our `cmd-*` Design System

We define a set of reusable component classes in `app.css` prefixed with `cmd-`:

- `cmd-btn`, `cmd-btn-sm` - buttons
- `cmd-btn-danger`, `cmd-btn-success`, `cmd-btn-primary` - button variants
- `cmd-btn-outline-danger`, `cmd-btn-outline-success` - outline variants
- `cmd-modal` - modal dialogs
- `cmd-list-item` - list items

These are defined in `<style scoped>` blocks or `app.css` and provide consistent styling across the GUI. Use Tailwind for layout, use `cmd-*` classes for component styling.
