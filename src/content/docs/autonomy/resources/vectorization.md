---
title: "Vectorization"
---

## What is vectorization and why do we need it? 
Python, unlike C++, is an interpreted programming language. This means that Python is not compiled down to raw machine code that runs on the CPU, but rather executed by another program (the interpreter). This is a key characteristic of Python and lends it the features that you know and love. The interpreter lets us run the same Python code on multiple platforms, execute Python code in an interactive shell, and enables dynamic typing. Unfortunately, for all this convenience we make a significant compromise in the speed of our programs. Interpreters are inherently slow because they come with significant overhead from machine instructions being determined runtime as opposed to compile time. This becomes a very serious bottleneck when trying to perform large amounts of numerical operations. 

Vectorization is a way to work around the speed limitations of Python: Rather than using native Python for loops to process data elements one at a time, we try to work with large batches of data in arrays. This is advantageous because there are libraries that implement parallel operations on arrays in C++ with Python bindings. This means that the underlying library code itself is written in multithreaded C++ so it is fast, but can be called from Python code with Python data. We specifically use the extremely ubiquitous NumPy library for mathematical computing. NumPy provides us with a range of functions that perform operations on arrays. These operations generally fall into a category like broadcasting, reduction, matrix operation, or element wise arithmetic.  

Other than speed, there are significant advantages to vectorized code. This style of programming lends itself particularly well to applications that are already making use of vector, matrix, or tensor-like data. In our case we frequently work with 2d vectors in navigation code. Vectoriztion is often considered idiomatic in robotics Python applications. It is useful to think about data as higher dimensional structures instead of individual elements. Vectorization can also lead to more compact code. Check out this Wikipedia article for more information: https://en.wikipedia.org/wiki/Array_programming.

## Vectorization example : spiral generation 
The following code snippets can both be used to generate square spirals like the one shown here: 
![176356593-a915270c-ebe6-49b2-b05e-c4568ac5abfd](https://user-images.githubusercontent.com/10037572/176975408-136e39d6-9d59-4545-ac06-bfd6f324170b.png)

**Before**
```python
out = []
cur = center
cur_segment_len = spacing / 2
directions = [np.array(1, 0), np.array(0, -1), np.array(-1, 0), np.array(1, 0)]
direction_index = 0
while cur_segment_len <= coverage_radius:
	next_point = cur + (cur_segment_len * directions[direction_index])
	out.append(next_point)
	cur_segment_len += spacing / 2
	direction_index = (direction_index + 1) % len(directions)
return out
```

**After**
```python
dirs = np.array([ [0, -1], [-1, 0], [0, 1], [1, 0] ])
deltas = np.tile(dirs, (num_turns, 1))
dist_coefs = distance * np.repeat(np.arange(1, num_turns*2+1), 2).reshape(-1, 1)
deltas *= dist_coefs
coordinates = np.cumsum(np.vstack((center, deltas)), axis=0)
return coordinates
``` 

## Code Walkthrough 
Start by reading and understanding the original code: it builds the spiral iteratively by applying some "delta" to the previous point in the spiral to get the next one.

The vectorized example applies the same strategy, but using numpy operations on 2d arrays. This makes it a nice example to study to understand how to vectorize something. However, keep in mind that often the optimal vectorized strategy for solving a problem looks nothing like its iterative counterpart. 

Below is the fully commented version of the code. Check out the numpy api docs [here](https://numpy.org/doc/stable/reference/index.html#reference). Try to understand what each line does. First get a conceptual understanding from the comment, then look up the numpy function that was used to understand how it is used to achieve the desired result. 

```python
# First we will attempt to create the "delta" vectors that get added add each point 
# in the spiral to get to the next. 
dirs = np.array([ [0, -1], [-1, 0], [0, 1], [1, 0] ])
deltas = np.tile(dirs, (num_turns, 1))
# We will build the coeficients for the delta vecs now that we have the correct
# layout of unit vectors Given the distance parameter 'd', the coef layout we 
# need is [d,d,2d,2d,3d,3d...]
dist_coefs = distance * np.repeat(np.arange(1, num_turns*2+1), 2).reshape(-1, 1)
deltas *= dist_coefs
# At this point we use cumsum to create a new array of vectors where each vector
# is the sum of all the previous deltas up to that index in the old array. We
# also make sure to add the center coordinate in here too so the spiral is in
# the correct location
coordinates = np.cumsum(np.vstack((center, deltas)), axis=0)
```

## Closing Remarks

In some ways, vectorizing this spiral example is overkill. In practice we will be generating spirals with 3-4 turns and we don't need the speed improvements from vectorization. However, in the interest of staying idiomatic, we want to use vectorized code anyway. Just for fun, lets evaluate the runtime performance of these two examples for a large spiral with 1000 turns: 
```
Time for generating 1000 turns: 
Before: 5.66 ms
After: 0.22 ms
```
As you can see, this improvement is dramatic. The vectorized code is upwards of 25 times faster. 