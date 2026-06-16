---
title: "Key Detection"
---
**Context**: During the ES mission we now need to be able to make the rover autonomously type on a keyboard. As this is a new task for this year, we will begin by researching and testing for different letter detection algorithms which could be run to detect letters on individual keys.

**Problem**: Detecting individual keys on a keyboard is a non-trivial task. Accordingly, it is perception's job to be able to detect/estimate different keys' locations in the real world. This information could then be passed off to the IK system to move the arm towards a desired location or in a desired direction.

**Solution**:

Rough Steps:
* Perform research for two different types of OCR
* Create an implementation for which these two implementations could be tested (Hopefully Done by CDR)
* Begin to identify strengths and weaknesses of each algorithm

<hr>

**Interface** (subject to change)

The interface will become more defined within the next week.