#ifndef CONFIG_H
#define CONFIG_H

#define GOAL 2 // 2 - yellow, 3 - blue

#if GOAL == 2
#define HOME 3
#endif
#if GOAL == 3
#define HOME 2
#endif

#endif