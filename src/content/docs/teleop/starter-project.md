---
title: "Teleop Starter Project"
---
# Introduction

This project will involve building an ArmControls component on a testing view, connecting that to the backend, then using the simulator and the THREE.js component to verify the implementation. 

Any spot marked with a ```TODO``` indicates that code should be added there. 

Please make sure you understand what you are adding when you add it and ask questions as you go. This project is solely for your learning so take as much time as you need to understand what is actually happening as you go through it.

# Getting Started
First, checkout the branch that has the starter project code: 
```bash
git checkout teleop-starter-new
```
Then checkout a new branch to work on:
```bash
git checkout -b <your initials>/starter-project
example: git checkout -b tjn/starter-project
```

Make sure that you have gone through the process detailed in [Teleop Quickstart](/teleop/quickstart) so that all dependencies are installed correctly

Since this starter project is just another blank page added to the codebase, you would launch the backend just like you normally would, with:

```bash
ros2 launch mrover basestation.launch.py
```

Then, open your browser, and go to ```http://localhost:8080/starter``` to verify that it has launched correctly. You should see the header, and the text "body" below. 

---

# Understanding Websockets

Websockets provide a persistent, two-way communication channel between the frontend and backend, Once the connection is established, messages are sent instantly with very low latency, which makes it suitable for live data applications like our frontend. 

---

# WebsocketStatus component

Notice the navigation bar, particularly the ```wypt``` websocket status indicator at the top right. As explained by the diagram, a green blinking dot indicates a transmit, and a red blinking dot indicates a receive. 

Now, in your terminal window where you previously launched the basestation, hit ```ctrl-c```. 

You will notice that the ```wypt``` indicator's background has turned yellow. A yellow background indicates a disconnect, either because of a websocket error or that we have shut down the backend. 

## Lets try out the functionality of this component!

Open ```/teleoperation/basestation_gui/frontend/src/views/StarterProject.vue```

Modify the ```<template>```: Replace the ```<h1>``` tag with a button. Connect the button to the method ```spamTestmessages()``` using ```@click```:

```html
<button class="btn btn-primary" @click="spamTestMessages">
  send websocket messages
</button>
```

Now, when you click the button, you should observe the websocket indicator for the ```wypt``` consumer flash green five times. 

Go back to the terminal window where you ran the basestation. You should observe the following message from the backend, which acknowledges that the message has been received by the django server:


```bash
[gui_backend.sh-1] [WARN] [1753735204.830010889] [gui_backend_node]: debug message received by consumer, 2025-07-28T20:40:04.827Z
```

You have successfully used the websocket to send a message to the backend!

---

# Importing and using components

Now, we'll import and develop the ```ArmControls``` and ```Rover3D``` components. 

Add import statements for ```ArmControls.vue``` and ```Rover3D.vue``` at the top of your ```<script>``` section:

```js
import ArmControls from '../components/ArmControls.vue'
import Rover3D from '../components/Rover3D.vue'
```

And register them in the component section. Remember that you can only use a component after it has been registered:

```js
components: {
    ArmControls,
    Rover3D,
},
```

Whenever you add new components to a top-level view, you should be aware of the websocket consumers that they utilize. 

:::important
To find which consumer handles a specific ROS2 topic or service, see [Consumers Lookup](/teleop/consumers-lookup)
:::

Looking in ```ArmControls``` And ```Rover3D```, you will see that they only use the ```arm``` websocket consumer. Therefore, we should initialize it in ```StarterProject.vue```:

in ```mounted()```:
```js
this.$store.dispatch('websocket/setupWebSocket', 'arm')
```

and in ```unmounted()```
```js
this.$store.dispatch('websocket/closeWebSocket', 'arm')
```
:::note
In Vuex, ```this.$store``` gives components access to the global store, which holds shared state and actions. In particular, we are using Vuex to store our websockets logic so that any component can access it by using ```this.$store.dispatch()```
:::

Now, you can incorporate these components into the ```<template>``` section, throwing them all under the ```view-wrapper``` div:

```html
<ArmControls class="island py-2" />
<Rover3D class="island m-0 p-0" style="max-height: 700px;" />
```

:::note
```island``` is a custom css class, defined to give a component rounded corners and a white background. Implementation could be found in ```App.vue```
:::

Looking at your browser window, you should realize that the current layout is not ideal, with the test button and ```ArmControls``` taking up full separate rows. Move the button and the control into the same row by using a div with classes ```d-flex flex-column```, and add spacing as appropriate. 

Your ```<template>``` section should now look like this

```html
<template>
  <div class="view-wrapper">
    <div class="d-flex flex-col gap-2 mb-2">
      <button class="btn btn-primary" @click="spamTestMessages">
        send websocket messages
      </button>
      <ArmControls class="island py-2" />
    </div>
    <Rover3D class="island m-0 p-0" style="max-height: 700px;" />
  </div>
</template>
```

and ```<script>``` section:

```js
<script lang="ts">
import ArmControls from '../components/ArmControls.vue'
import Rover3D from '../components/Rover3D.vue'
import { defineComponent } from 'vue'
import Vuex from 'vuex'
const { mapState } = Vuex
import type { WebSocketState } from '../types/websocket'

export default defineComponent({
  components: {
    ArmControls,
    Rover3D,
  },

  mounted() {
    this.$store.dispatch('websocket/setupWebSocket', 'arm')
    this.$store.dispatch('websocket/setupWebSocket', 'waypoints')
  },

  unmounted() {
    this.$store.dispatch('websocket/closeWebSocket', 'arm')
    this.$store.dispatch('websocket/closeWebSocket', 'waypoints')
  },

  computed: {
    ...mapState('websocket', {
      waypointsMessage: (state: WebSocketState) => state.messages['waypoints'],
    }),
  },

  watch: {
    waypointsMessage(msg) {
      console.log(msg)
    },
  },

  methods: {
    spamTestMessages() {
      const interval = setInterval(() => {
        this.$store.dispatch('websocket/sendMessage', {
          id: 'waypoints',
          message: {
            type: 'debug',
            timestamp: new Date().toISOString(),
          },
        })
      }, 1000)
      setTimeout(() => clearInterval(interval), 5000)
    },
  },
})
</script>
```

Check your browser window to see how it looks. Pretty neat, isnt it?

---

# Developing ```ArmControls```

Now open up ```ArmControls.vue``` in your preferred text editor, located at ```/teleoperation/basestation_gui/frontend/src/components/ArmControls.vue```. The core functionality in this component has already been implemented. 

## Mode Selector

First, lets make a button group to modify the **mode** variable. Add a div with the bootstrap group ```btn-group``` below the ```<div>``` containing the ```<h3>```

```html
<div
class="btn-group d-flex"
role="group"
aria-label="Arm mode selection"
>

</div>
```

Inside this div, place two buttons, one selecting ```disabled```, and one for ```throttle```, the simplest controller mode that maps each joint of the arm to an axis:

```html
<button
    type="button"
    class="btn flex-fill"
    :class="mode === 'disabled' ? 'btn-danger' : 'btn-outline-danger'"
    @click="mode = 'disabled'"
>
    Disabled
</button>
<button
    type="button"
    class="btn flex-fill"
    :class="mode === 'throttle' ? 'btn-success' : 'btn-outline-success'"
    @click="mode = 'throttle'"
>
    Throttle
</button>
```

As you can now see from your browser, the ```btn-group``` creates a joined effect on the buttons, the ```@click``` modifies the ```mode``` variable when selected, and the ```:class``` styling makes the button fill solid when selected. 


## Send Controls

The original code requires a **controller** to be connected in order to send controls. Here, we have modified the code using the **WASD** keys to mimic the left joystick of the controller, like so

```js
axes[0] = (this.keysPressed.d ? 1 : 0) - (this.keysPressed.a ? 1 : 0)
axes[1] = (this.keysPressed.s ? 1 : 0) - (this.keysPressed.w ? 1 : 0)
```

**axes** and **buttons** are used to simulate an xbox controller. We will now send both the mode and the controls to the ```arm``` consumer. 

There is already an interval set up in ```created()```, which repeats 30 times a second. As mentioned earlier, to access and use websockets, we will use ```this.$store.dispatch()```:

```js
this.$store.dispatch('websocket/sendMessage', {
    id: 'arm',
    message: {
        type: 'ra_controller',
        axes: axes,
        buttons: buttons,
    },
})
this.$store.dispatch('websocket/sendMessage', {
    id: 'arm',
    message: {
        type: 'ra_mode',
        mode: this.mode,
    },
})
```

---

## Verify commands

Open another terminal window, enter ```mrover```, then enter 

```
ros2 topic list
```

This should list all the topics which are live, which includes ```/arm_throttle_cmd``` which we are publishing to

Now, enter ```ros2 topic echo /arm_throttle_cmd```. Place the window along side your browser window, focus on the browser, select ```throttle```, and press the WASD keys. You should see the published values react to your keystrokes. 

This is the method used to debug ROS topic. 

---

## Completed ```ArmControls.vue```

Now, your ```<template>``` section should look like this:

```html
<template>
  <div class="d-flex flex-column align-items-center w-100">
    <div class="d-flex flex-column gap-2" style="width: 500px; max-width: 100%">
      <div class="d-flex justify-content-between align-items-center">
        <h3 class="m-0">Arm Controls</h3>
      </div>
      <div
        class="btn-group d-flex"
      >
        <button
          type="button"
          class="btn flex-fill"
          :class="mode === 'disabled' ? 'btn-danger' : 'btn-outline-danger'"
          @click="mode = 'disabled'"
        >
          Disabled
        </button>
        <button
          type="button"
          class="btn flex-fill"
          :class="mode === 'throttle' ? 'btn-success' : 'btn-outline-success'"
          @click="mode = 'throttle'"
        >
          Throttle
        </button>
      </div>
    </div>
  </div>
</template>
```

and ```<script>```:

```js
<script lang="ts">
import { defineComponent } from 'vue'
import Vuex from 'vuex'
const { mapActions } = Vuex

const UPDATE_HZ = 30

export default defineComponent({
  data() {
    return {
      mode: 'disabled',
      keysPressed: {
        w: false,
        a: false,
        s: false,
        d: false,
      },
      interval: 0,
    }
  },
  mounted() {
    document.addEventListener('keydown', this.handleKeyDown)
    document.addEventListener('keyup', this.handleKeyUp)
  },
  created() {
    this.interval = window.setInterval(() => {
      const axes = [0, 0, 0, 0]
      const buttons: boolean[] = []
      axes[0] = (this.keysPressed.d ? 1 : 0) - (this.keysPressed.a ? 1 : 0)
      axes[1] = (this.keysPressed.s ? 1 : 0) - (this.keysPressed.w ? 1 : 0)

      this.$store.dispatch('websocket/sendMessage', {
        id: 'arm',
        message: {
          type: 'ra_controller',
          axes: axes,
          buttons: buttons,
        },
      })
      this.$store.dispatch('websocket/sendMessage', {
        id: 'arm',
        message: {
          type: 'ra_mode',
          mode: this.mode,
        },
      })
    }, 1000 / UPDATE_HZ)
  },
  beforeUnmount() {
    window.clearInterval(this.interval)
    document.removeEventListener('keydown', this.handleKeyDown)
    document.removeEventListener('keyup', this.handleKeyUp)
  },
  methods: {
    ...mapActions('websocket', ['sendMessage']),
    handleKeyDown(event: KeyboardEvent) {
      const key = event.key.toLowerCase()
      if (key === ' ') {
        this.mode = 'disabled'
      }
      if (key in this.keysPressed) {
        this.keysPressed[key as keyof typeof this.keysPressed] = true
      }
    },
    handleKeyUp(event: KeyboardEvent) {
      const key = event.key.toLowerCase()
      if (key in this.keysPressed) {
        this.keysPressed[key as keyof typeof this.keysPressed] = false
      }
    },
  },
})
</script>
```

---

# Testing

In another terminal window, enter ```mrover```, then 

```
ros2 launch mrover simulator.launch.py
```

This will launch the sim and allow us to test our ```ArmControls``` component. 

In the newly launched simulator window (the one with the 3D world), press ```esc``` to unlock your cursor, and press ```p``` to enable physics. Then, uncheck the ```Publish IK``` checkbox on the left sidebar. 

Back in your browser, click the ```Throttle``` button in ```ArmControls```. 

Now, when you press the ```**WASD**``` keys, you should see the ```Rover3D``` move, responding to the rover's joints in the simulator. 


---

# Congratulations, you've finished the starter project!
