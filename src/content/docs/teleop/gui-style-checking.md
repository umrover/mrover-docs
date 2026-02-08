---
title: "GUI Style Checking"
---
## Style Teleop Code

Keeping our code well-formatted makes it easier for everyone to read and maintain. 

**Always format your code before pushing commits,** or at least try to remember

### Backend ROS Code (Python/C++)

From the root of the `mrover` repo, run:

```bash
./style.sh --fix
```


- Running `./style.sh` by itself will **show files that need formatting**.
- Running with `--fix` will **automatically apply formatting**


### Frontend Code (Vue/JavaScript/HTML)

Navigate to the frontend directory (from root ```mrover```):

```bash
cd teleoperation/basestation/frontend
```

Then run the following:

```bash
bun run lint
```

- This runs **ESLint**, which checks for code style violations and common bugs in your JavaScript and Vue code.
- If you see any lint errors, **fix them before pushing** your code.

Additionally, the **Prettier** plugin should take care of the mode mundane tasks like spacing and indentation in your code when you save (ctrl-s)

These are new rules that we haven't strictly enforced before, so don't be surprised if you find old chunks of code which haven't been formatted. 

---