---
categories:
- programming, cpp
date: '2024-03-11'
description: My notes for cpp
layout: post
title: Cpp Common Knowledge
toc: true

---

# CPP some questions

Some questions for my CPP learning and its answers

## 1. Why add #ifndef and #define _HEADER_H in the top of header files ?

### Preventing Multiple Inclusions:

Header files often contain declarations for functions, variables, classes, etc., that multiple source files might need.
Without #ifndef and #define, including the same header file twice in a source file would lead to multiple definitions, causing compilation errors.

Example:

```code
#ifndef MY_HEADER_H  // Check if MY_HEADER_H is not defined
#define MY_HEADER_H  // Define MY_HEADER_H to prevent re-inclusion

// Header contents (functions, variables, etc.)

#endif               // End of conditional inclusion
```

### Benefits:

- Ensures header files are included only once, preventing compilation errors.
- Makes code more maintainable and avoids unexpected behavior.

# 2. 
