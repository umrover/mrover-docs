---
title: "Sample Vue Component"
---
## Sample Vue Component
This code block contains a sample vue component that has examples and explanations of many different paradigms that you will use when developing GUI components with Vue.

```vue
<!-- Define the html of your components here -->
<!-- Note that there can only be one top level div in the template -->
<template>

    <!-- CSS Classes can be added to a div using class="class1 class2" -->
    <div class="wrapper box">

        <!-- You can use v-for to render items in a loop -->
        <div v-for="i in x">
            <!-- Event handlers can be declared with v-on:{eventName} -->
            <button v-on:click="addOnetoX()">Add one to x #{{i}}</button>
        </div>

        <!-- v-for can also be used to index through data structures like arrays -->
        <!-- similar to Python -->
        <div v-for="elt in arr">
            <!-- Use {{}} to display data on the DOM -->
            {{elt}}
        </div>

        <!-- v-if can be used to conditionally render html -->
        <div v-if="x > 2">
            <p> x is greater than two</p>
        </div>

    </div>
</template>
  
<!-- Here you can define javascript functionality for your component -->
<script>

  // Define imports for other components and packages here
  import { mapGetters } from 'vuex'
  import Component from './Component.vue'


  // Define const variables here
  // These will only exist in the scope of the <script> tag
  const two = 2
  
  // Must include "this." in front of any var or function 
  // defined inside of export section 
  export default {
    // Can be used for recursively rendering a component
    // Can also be helpful for debugging
    name: 'AutonTask',
    // Define component local variables
    data () {
      return {
        x: 1,
        arr: [1, 2, 3]
      }
    },
  
    // These are general functions that can be used 
    methods: {
        addOnetoX: function () {
            this.x = this.x + 1
        }
    },
  
    // Values that can be treated like read-only variables
    // Calculated based on component member variables
    // Note the this in front of var x but not const two
    computed: {
        xPlusTwo: function () {
            return this.x + two
        }
    },
  
    // Functions that have the same name as a variable and will run
    // Whenever the value of that var changes
    watch: {
    // Runs whenever variable x changes
    // val is the new value that x just changed to
      x: function(val){
        console.log("x has changed")
      }
    },
  
    // Run before the component is created, set up any logic essential for 
    // variables in the component
    beforeCreate: function () {

    },

    // Function that will run at the creation of the Component but before it is rendered to the DOM 
    created: function () {

    },

    // Function that runs immediatly after rendering component to DOM
    mounted: function () {

    },

    // Function that runs after any update to the DOM
    updated: function (){

    },

    // Function that runs after DOM is unrendered
    // Use this for any cleanup functions or to emit 
    // an event about the destruction of the components
    unmounted: function () {

    },
  
    // Child Components that will be used to assemble the current component
    components: {

    }
  }
</script>

<!-- This is where you will define CSS Classes for your components -->
<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
/* Examples of CSS classes */
.wrapper{
    margin: 1px
}

.box{
    border: 1px grey
}

</style>
  
```