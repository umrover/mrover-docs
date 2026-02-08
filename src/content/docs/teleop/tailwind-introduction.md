---
title: "Tailwind Introduction"
---
# Tailwind Introduction

## About CSS Frameworks

CSS frameworks like **Bootstrap** and **TailwindCSS** are designed to reduce the need for writing separate ```.css``` files. Instead, styles can be applied directly in your HTML using predefined classes, making development more convenient.

Although this approach can be powerful, we don’t fully adopt it in this project to keep the learning curve low. Since Vue components combine HTML, JavaScript, and CSS in one file, managing styles is already relatively convenient.

That said, before jumping into ```<style scoped>``` to write something like:

```css
margin-bottom: 1rem;
```

You should consider using the spacing utility instead:

```html
<div class="mb-3"></div>
```

This keeps your code cleaner and more consistent with the rest of the project.

## Spacing Utility
Throughout the codebase, you’ll see classes like ```px-2``` and ```m-3```. These are Tailwind utility classes that control padding and margin. They provide a fast and readable way to apply spacing without writing custom CSS.


### Property - first letter
- `m` – margin
- `p` – padding

### Sides Selection - second letter
- `t` – top
- `b` – bottom
- `s` – start
- `e` – end
- `x` – left and right (horizontal)
- `y` – top and bottom (vertical)
- (omitted) – all sides

### Size (spacing)
- `0` – 0 spacing
- `1` – 0.25rem
- `2` – 0.5rem
- `3` – 1rem
- `4` – 1.5rem
- `5` – 3rem
- `auto` – auto margin (only for `m`)

### What is a `rem`:
`rem`, or **r**oot **e**le**m**ent, is a measurement that corresponds to the font size of the root element. By default in most browsers:

```css
html {
    font-size: 16px;
}
```

Therefore, in this case, `1rem = 16px`, `0.5rem = 8px`, etc. 

### Some Examples

| Class       | Description                          |
|-------------|--------------------------------------|
| `m-3`       | Margin on all sides = 1rem           |
| `pt-2`      | Padding top = 0.5rem                 |
| `py-0`      | Padding top & bottom = 0             |
| `ms-auto`   | Margin start = auto                  |