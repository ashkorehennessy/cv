//
// Created by ashkore on 25-2-27.
//

#ifndef CONFIG_H
#define CONFIG_H
#define DISTANCE(x1, y1, x2, y2) sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2))
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define ABS(x) ((x) > 0 ? (x) : -(x))
#endif //CONFIG_H
