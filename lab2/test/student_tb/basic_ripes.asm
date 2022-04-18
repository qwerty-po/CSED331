<main>:
        addi    sp,sp,-32
        sw      ra,28(sp)
        sw      s0,24(sp)
        addi    s0,sp,32
        addi    a5,zero,19
        sw      a5,-20(s0)
        addi    a5,zero,14
        sw      a5,-24(s0)
        lw      a1,-24(s0)
        lw      a0,-20(s0)
        sw      a0,-28(s0)
        lw      a4,-28(s0)
        lw      a5,-20(s0)
        add     a5,a4,a5
        ori     a6,zero,10
        addi    a7,zero,2
        sll     a7,a6,a7
        addi    a1,zero,3
        and     a3,a7,a1
        not     a2,a7
        sub     a6,a7,a6
        srli    a7,a7,3
        or      a3,a4,a5
        lw      ra,28(sp)
        lw      s0,24(sp)
        addi    sp,sp,32
        li      a7, 10
        ecall
