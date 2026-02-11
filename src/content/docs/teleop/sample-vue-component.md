---
title: "Sample Vue Component"
---
## Sample Vue Component

This code block contains a sample Vue component using the Composition API with `<script setup>`. It demonstrates the patterns you will use when developing GUI components.

```vue
<template>
  <div class="flex flex-col gap-2">

    <!-- v-for loops through reactive arrays -->
    <div v-for="i in count" :key="i">
      <button class="cmd-btn cmd-btn-primary" @click="increment">
        Add one to count (#{{ i }})
      </button>
    </div>

    <!-- v-for with arrays -->
    <div v-for="item in items" :key="item">
      {{ item }}
    </div>

    <!-- v-if for conditional rendering -->
    <p v-if="count > 2">Count is greater than two</p>

    <!-- computed properties are used like regular values -->
    <p>Count plus two: {{ countPlusTwo }}</p>

  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch, onMounted, onBeforeUnmount } from 'vue'

// reactive state
const count = ref(1)
const items = ref([1, 2, 3])

// methods are just functions
function increment() {
  count.value++
}

// computed: derived values that auto-update
const countPlusTwo = computed(() => count.value + 2)

// watch: run code when a value changes
watch(count, (newVal) => {
  console.log('count changed to', newVal)
})

// lifecycle: runs after component mounts to DOM
onMounted(() => {
  console.log('component mounted')
})

// lifecycle: cleanup before component is removed
onBeforeUnmount(() => {
  console.log('component unmounting')
})
</script>

<style scoped>
/* component-specific styles go here */
</style>
```

### Key differences from the Options API

If you see older code using `export default { data(), methods, computed, watch }`, that is the **Options API**. We now use the **Composition API** with `<script setup>`:

| Options API | Composition API |
|------------|----------------|
| `data() { return { x: 1 } }` | `const x = ref(1)` |
| `methods: { fn() {} }` | `function fn() {}` |
| `computed: { val() {} }` | `const val = computed(() => ...)` |
| `watch: { x(val) {} }` | `watch(x, (val) => ...)` |
| `mounted() {}` | `onMounted(() => {})` |
| `this.x` | `x.value` |
| `components: { Foo }` | Just `import Foo from ...` |
| `this.$store.dispatch(...)` | `const store = useXxxStore()` |
