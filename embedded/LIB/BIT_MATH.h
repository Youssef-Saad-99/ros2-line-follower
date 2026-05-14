#ifndef BIT_MATH_H
#define BIT_MATH_H

#define SET_BIT(REG,BIT) (REG|=1<<BIT)
#define CLR_BIT(REG,BIT) (REG&=~(1<<BIT))
#define TOGGLE_BIT(REG,BIT) (REG^=1<<BIT)
#define GET_BIT(REG,BIT)   ( ((REG) >> (BIT)) & 1 )
#define SWAP_BITS(REG, BIT1, BIT2) (                            \
    (( ((REG) >> (BIT1)) & 1 ) != ( ((REG) >> (BIT22)) & 1 )) ?  \
    ((REG) ^ (1 << (BIT1)) ^ (1 << (BIT2))) : (REG)             \
)

#endif
