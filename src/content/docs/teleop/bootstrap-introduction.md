---
title: "Bootstrap Introduction"
---
# Bootstrap Introduction

Bootstrap is a css framework that makes css styling a whole lot easier for you. In this codebase, bootstrap's spacing utility is frequently used. Click [here](https://getbootstrap.com/docs/5.3/getting-started/introduction/) to access Bootstrap docs. 

## About CSS Frameworks

CSS frameworks like **Bootstrap** and **TailwindCSS** are designed to reduce or completely eliminate the need for writing separate ```.css``` files. Instead, styles can be applied directly in your HTML using predefined classes, making development more convenient.

Since Vue uses SFCs (Single File Components), and combines HTML, Javascript, and CSS all in one file, using ```<style scoped>``` makes styling already a lot more convenient. 

That said, before jumping into ```<style scoped>``` to write something like:

```css
margin-bottom: 1rem;
```

You should consider using Bootstrap’s spacing utility instead:

```html
<div class="mb-3"></div>
```

To keep our codebase clean and consistent, **always prefer Bootstrap’s utility classes** (like `mb-3`, `text-center`, `d-flex`, etc.) over writing custom CSS in `<style scoped>`. These utilities cover most common styling needs such as spacing, alignment, colors, and layout.

Use `<style scoped>` **only when**:

- You need to define **custom styles** not covered by Bootstrap (e.g., a unique font family or a custom color),
- Or when **overriding** Bootstrap defaults in a controlled and component-specific way.

This approach helps us:

- Maintain a uniform design language across the app
- Reduce redundant CSS
- Speed up development

## Bootstrap Spacing Utility
Throughout the codebase, you’ll see classes like ```px-2``` and ```m-3```. These are Bootstrap utility classes that control padding and margin. They provide a fast and readable way to apply spacing without writing custom CSS.

Click [here](https://getbootstrap.com/docs/5.3/utilities/spacing/#margin-and-padding) to read more, below is a short summary. 

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