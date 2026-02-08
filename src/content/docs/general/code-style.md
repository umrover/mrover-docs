---
title: "Code Style"
---
### Formatting
Run `./style.sh` in the top-level directory. `./style.sh --fix` will attempt to auto-format.

### General
- Prefer longer variable names. Avoid abbreviations. We have autocomplete for a reason!

### C++
- Adhere to all clang tidy checks defined in `.clang-tidy`. **This is tested by the CI.**
- Style syntax with the clang format defined by `.clang-format`. **This is tested by the CI.**
- *Never use raw pointers*, prefer [smart pointers](https://en.cppreference.com/book/intro/smart_pointers) (usually `std::shared_ptr`).
- Check if something exists in the standard library before you reinvent the wheel!
- Prefer [`static_cast`](https://stackoverflow.com/questions/332030/when-should-static-cast-dynamic-cast-const-cast-and-reinterpret-cast-be-used) etc. instead of C style cast
- Use `auto` on iterator types (ranged for) and *when the type is obvious*, for example casting or `std::make_shared`
- Pass primitives by value (`int`, `float`, etc.) and complex types by reference or smart pointers
- Do not use references (&) types for out parameters in functions, prefer returning a struct
- Prefer using an explicit struct instead of `std::pair` or `std::tuple`
- Use [structured binding](https://en.cppreference.com/w/cpp/language/structured_binding) when possible (e.g. iterating named map, unpacking a `std::pair`, ...)
- Avoid `std::bind`, use an explicit lambda
- Prefer PascalCase for things like classes and camelCase for things like member variables. **This is tested by the CI.**

### Python
- Currently `black` and `mypy` are used. **This is tested by the CI.**
- If you are writing a function, *please use [type hinting](https://docs.python.org/3/library/typing.html) for the signature*. For other places, only use if necessary, for example a situation where the types are confusing.
- Black is an autoformating tool that will control the format of your code
- MyPy is a static type analysis tool that will make sure you don't violate type hinted functions
- Document your code! Use the reST standard specifically
- Prefer snake_case for regular python classes, functions, and variables and UPPER_CASE for constants

Here is a good concrete example:
```python
def pos_distance_to(self, p: SE3) -> float:
    """
    Get the euclidean distance from the position of this SE3 pose to the position of another SE3 pose.
    :param p: another SE3 pose object
    :returns: euclidean distance between the two SE3 poses
    """
    return np.linalg.norm(p.position_vector() - self.position_vector())
```
