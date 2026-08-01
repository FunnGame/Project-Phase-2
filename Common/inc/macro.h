
#ifndef MACRO_H
#define MACRO_H

#ifdef __cplusplus
extern "C" {
#endif


#define SET_BIT(REG, BIT)          ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)        ((REG) &= ~(BIT))
#define TOGGLE_BIT(REG, BIT)       ((REG) ^= (BIT))
#define READ_BIT(REG, BIT)         ((REG) & (BIT))

#define WRITE_BIT(REG, BIT, VAL)   \
    ((VAL) ? SET_BIT(REG, BIT) : CLEAR_BIT(REG, BIT))


#define MIN(A, B)                  (((A) < (B)) ? (A) : (B))
#define MAX(A, B)                  (((A) > (B)) ? (A) : (B))
#define ABS(X)                     (((X) < 0) ? -(X) : (X))
#define CLAMP(X, MIN_VAL, MAX_VAL) \
    (((X) < (MIN_VAL)) ? (MIN_VAL) : (((X) > (MAX_VAL)) ? (MAX_VAL) : (X)))


#define ARRAY_SIZE(ARRAY)          (sizeof(ARRAY) / sizeof((ARRAY)[0]))
#define UNUSED(X)                  ((void)(X))


#define ENABLE                     (1)
#define DISABLE                    (0)

#define HIGH                       (1)
#define LOW                        (0)

#define ON                         (1)
#define OFF                        (0)

#ifdef __cplusplus
}
#endif

#endif /* MACRO_H */