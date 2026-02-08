---
title: "Vue Introduction"
---
# Vue Introduction

Vue is a versatile and beginner-friendly JavaScript framework. This introduction is only going to cover what is relevant to our codebase. 

## File Structure

A typical `.vue` file includes;
- A `<template>` block for HTML markup
- A `<script>` block for js logic
- A `<style>` block for css styling

Like this:
```vue
<template>
  <div class="wrapper">
      <!-- Child content here -->
  </div>
</template>
<script>
  // Logic
</script>
<style scoped>
  /* Styles */
</style>
```

> [!NOTE]   
> `<style scoped>` limits the styling to the current component 

## Looping Through Data: `v-for`

frontend/src/components/AutonWaypointEditor.vue
```vue
<WaypointStore
  v-for="(waypoint, index) in waypoints"
  :key="waypoint"
  :waypoint="waypoint"
  :index="index"
  @add="addItem"
  @delete="deleteMapWaypoint"
/>
</div>
```

## Conditional Rendering: `v-if`

**Only render the element if the condition is true.**

frontend/src/components/AutonWaypointItem.vue

```vue
<button
  v-if="!enable_costmap"
  class="btn btn-danger"
  @click="toggleCostmap"
>
  Costmap
</button>

<button
  v-if="enable_costmap"
  class="btn btn-success"
  @click="toggleCostmap"
>
  Costmap
</button>
```

## Event Binding: `v-on` / `@`

**Bind a function to an event like `click`.**

frontend/src/components/CameraFeed.vue

```vue
<canvas :id="'stream-' + id" v-on:click="handleClick"></canvas>
```

## Data Properties

**Define component-local variables using the `data()` function.**

```js
data() {
  return {
    x: 1,
    arr: [1, 2, 3]
  }
}
```

## Methods

**Define functions you want to call from your template or internally.**

```js
methods: {
  addOnetoX() {
    this.x = this.x + 1
  }
}
```

## Computed Properties

**Used for derived or reactive values based on existing data.**

```js
computed: {
  xPlusTwo() {
    return this.x + 2
  }
}
```

## Watchers

**Run code in response to changes in a specific data property.**

```js
watch: {
  x(val) {
    console.log("x has changed")
  }
}
```

## Lifecycle Hooks

### `beforeCreate()`

Runs before the component is initialized.

```js
beforeCreate() {
  // setup logic here
}
```

### `created()`

Runs after the component is initialized, but before DOM is mounted.

```js
created() {
  // fetch data, set up reactive properties, etc.
}
```

### `mounted()`

Runs after the component is added to the DOM.

```js
mounted() {
  // DOM-dependent logic
}
```

### `updated()`

Runs after any DOM update caused by reactive changes.

```js
updated() {
  // respond to reactive updates
}
```

### `unmounted()`

Runs when the component is removed from the DOM. Great for cleanup.

```js
unmounted() {
  // cleanup logic, remove event listeners
}
```

---

## Importing Other Components or Utilities

**Import other files for reuse.**

```js
import Component from './Component.vue'
```

> [!WARNING]  
> Ever since Vuex 4, instead of this, which gives a warning
```
import { mapGetters } from 'vuex'
```
You should do this instead
```
import Vuex from 'vuex'
const { mapGetters } = 'vuex'
```

---

## Child Components

Register child components that you want to use inside this component. If you don't register it, Vue won't recognize it. 

```js
components: {
  Component
}
```


## See [here](/teleop/sample-vue-component) for a complete example 