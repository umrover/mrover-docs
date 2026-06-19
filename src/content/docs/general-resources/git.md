---
title: "Git"
---
##### Git commits:

Git commits are basically diffs between files that exist on a github repository. These act as fixes or changes to files and reduce the total amount of information that git has to store.

Helpful Syntax:

`git commit -m "<message>"`: Git requires that each commit has a message describing what is contained inside of each commit. The `-m` flag is helpful because it allows one to include a message from the command line rather than using an editor.

##### Git Branches:

Git branches act as pointers to specific commits. Git branches include all of the commits that occurred on this branch + all of the commits that were on the parent branch.

Helpful Syntax:

`git branch <branch name>`: Can be used to create a new branch, but this **will not change the user to be on this current branch**.

`git switch [<branch name>, -]` Can be used to switch the user to a branch titled `<branch name>`, or switch the user to whichever branch they were on before this one.

`git branch -f <branch name> commit` Can be used to move where a branch is pointing to a specific commit.

##### Git Merging:

Git merging essentially takes the current branch that you are working on and adds all of the commits (and their corresponding deltas) to this branch. The way git accomplishes this is by:
1. Finding the earliest common ancestor for both of the git branches.
2. Calculating the deltas between the tips of each of the branches that are being merged and their earliest common ancestor.
3. Combining the deltas of each of the branches and add these changes in a new merge commit. If there are conflicts (ie. the same lines have deltas on both of these branches), then git will wrap these deltas in a merge conflict and make the user manually resolve them.

Helpful Syntax:

`git merge <branch name>`: Can be used to merge `<branch name>` into the branch the user is currently on.

##### Git Rebasing:

Git rebasing essentially takes the current branch and pastes the new commits it has after the new commits on another branch. In essence it makes the commit history much cleaner as features worked on in parallel look as though they were developed sequentially.

Similar to merge conflicts, rebase conflicts can occur when rebasing onto a branch that has edited the same lines as the current branch. When this happens just resolve the conflicts, add them back, and continue the rebase.

Helpful Syntax:

`git rebase <branch name>`: Can be used to rebase the current branch after `<branch name>`.

`git rebase --continue`: Can be used to continue the automatic rebase after rebase conflicts have been resolved.

`git rebase --abort`: Can be used to stop and discard all of changes being made by the current rebase.

##### Miscellaneous:

Detached Head: A user's HEAD points towards the current commit they are on. When a user is on a git branch their HEAD points to the same commit as the branch. However, a user can detach their head and point to different commits as necessary using the `switch` and `checkout` features.

Helpful Syntax:

`git switch --detach <commit hash>`: Can be used to change the user's HEAD to point toward the commit specified by `<commit hash>`. 

`HEAD~<N>`: Can be used in the place of a commit hash to reference a commit `<N>` commits back from the current head on this branch.

`HEAD^`: Can be used in place of `HEAD~1` to go up one commit.

Undoing Commits:

There are two ways to undo things in a github repository:

`git reset <commit hash>`: Can be used to reset the head of a current repository back to a given location specified by commit. Using this command has a disadvantage in that it is rewriting the git commit history and altering where the head of the branch is located.

`git revert <commit hash>`: Can be used to restore the specified commit. However, this is preferred over the `reset` because it will not rewrite the commit history and instead it will add a new commit undoing all of the changes things the desired commit.

## MRover Git Workflow:

##### Git Postulates:
- People should get credit for their PRs on `main`
- Development should not be hindered by the git workflow
- `main` should build and pass test cases
- git should be kept lightweight (no files above 1 MB)

##### Definitions:
- Fully functional: A piece of code that is fully working to the best of the developers abilities:
	- Passing smoke test cases
	- Passing regression test cases
	- Maintains basic functionalities and interfaces
- Checkpoint: A cohesive snapshot of the code base which is can be restored easily. For example:
	- Competitions
	- mock missions
	- major integration points
- Temporary branch: Temporary branches act as an integration platform for but interconnected features. **FEATURE BRANCHES SHOULD NOT BE ACTIVELY DEVELOPED ON** (this means temporary branches should only ever have merge commits on them).

##### Git Workflow:
- Releases:
  - Releases will act as *checkpoints* for previous, fully working states of the code.
  - When creating a release, a tag will be created which will act as a marker to which the code can be restored.
  - Decoding tags: \<competition year\>.\<major change\>.\<minor change\>
- `main` branch:
  - `main` will act as a place to merge *fully functional* code. 
  - Code can be integrated with main through the use of PRs (discussed later).
- Integration/Testing:
  - Integrating code will be done through the use of *temporary branches*. 
  - Development and integration can be done locally on a *temporary branch*, however when it comes time to commit these changes there are a few options. The first option is to switch to the feature branches where the changes belong and commit them there, instead of pushing them to the new **temporary** integration branch. The second option is to keep them local, though this choice is risky as they could be deleted later. The third option is to delete/restore them outright.
  - For example, consider I am testing the auton stack and have finished testing perception and navigation. I have changes to `navigation.py`, `zed_wrapper.cpp`, and `utils.ts`. I would like to keep the changes that I made to `navigation.py` and `zed_wrapper.cpp` and delete the changes I made to `utils.ts`. To integrate this properly I should run: 
    ```
    git switch nav
    git add navigation.py
    git commit -m "auton testing nav changes"
    git push
    git switch percep
    git add zed_wrapper.cpp
    git commit -m "auton testing percep changes"
    git push
    git restore utils.ts
    ```
- Pull Requests:
  - The expectation for `main` is that it is no longer a fully coherent piece of code. Thus, these relaxed constraints should allow code will be iterated faster and merged quicker, meaning more frequent and smaller pull requests. It is expected that any code being merged into `main` will have thorough testing associated with it, or else it will be rejected. 
  - These PRs should follow the *fully functional* ideology. A critical component of proving your code is *fully functional* is a robust set of tests to back this claim up.
  - To ensure continued correctness the CI pipeline must pass in order for a PR to be merged.
  - Dependent PRs, one which heavily rely on another distinct but interconnected feature, will be merged in simultaneously in order to allow users to use these new changes when pulling them in.
  - After PRs are made it is expected that everyone will merge `main` back into their feature branches.

Justification:

Why should `main` not be a checkpoint: There are two reasons why `main` should not be maintained as a checkpoint. First, it hinders development speed. If code must be fully integrated before merging into `main` then there is a much larger cost in terms of validation and PR review that must be taken into account. This can lead to large backups and slow iteration times, preventing code from being merged in at all. This will clearly hinder development iteration and violate postulate two. 

Second, if `main` is needed to be kept as a checkpoint, it becomes hard to accept someone's fully functional PR which has not been fully integrated. This means that code will be merged in to another branch, like `integration`, and integrated there. This puts too much emphasis on the PR into the integration branch, essentially making it the new `main`. Now, after all of the integration has been completed, the integration branch now has all of the desired features on it, meaning it makes more sense to merge the integration branch into `main`, rather than the original feature branches. Merging in `integration` rather than the feature branches violates postulate one because the original developers will not get to PR their feature branches into `main`. 

Why use github releases: Previously, we used `main` as our _checkpoint_ and ensured that any code merged in was fully functional. However, since this is no longer a requirement for `main` and we still need to maintain where the code base was fully functional, we can use the tag functionality inside of git in order to maintain these markers for us. The release feature on github will accomplish this because it will zip the code base and create a tag pointing at the specific commit.

Why should follow this integration strategy: The underlying reason for this integration strategy is to isolate changes that belong to a specific feature on its respective feature branch. First, the reason we should not directly commit changes to integration branches is because it will make it difficult to separate them into their respective branches later when it come time to PR everything into `main`. Therefore, it make logical sense to commit these changes to their respective feature branches instead so when it come time to PR everything, all of the changes for a specific feature are isolated on each branch. Thus, the rule "there should be no direct commits to a integration branch" makes sense. 

Why should we accept PRs which have not been fully integrated: As alluded to above in the "Why should `main` not be a checkpoint" paragraph, the main reason is iteration time. If we spend so much time trying to find all of the things that could go wrong with a given piece of code instead of actually finding out the actual problems with our code, we will struggle to keep up with a fast iteration cycle. However, there are concerns about accepting buggy code which can cause unexpected failures. The remedy for this is a robust test suite. Therefore, putting this strategy into practice will rely heavily on a robust regression test suite in order to ensure that expected functionality is not broken. Wrapping this func