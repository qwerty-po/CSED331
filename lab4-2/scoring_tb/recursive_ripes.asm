<main>:
        addi    sp,sp,-32
        sw      ra,28(sp)
        sw      s0,24(sp)
        addi    s0,sp,32
        addi    a5,zero,7
        sw      a5,-20(s0)
        lw      a0,-20(s0)
        jal ra, <fib>
        sw      a0,-24(s0)
        addi    zero,zero,0
        lw      ra,28(sp)
        lw      s0,24(sp)
        addi    sp,sp,32
        addi a6, a5, 8
        add s5, a6, a5
        add s7, s5, a6
        add s8, s5, s7
        li a7, 10
        ecall

<fib>:
        addi    sp,sp,-32
        sw      ra,28(sp)
        sw      s0,24(sp)
        sw      s1,20(sp)
        addi    s0,sp,32
        sw      a0,-20(s0)
        lw      a4,-20(s0)
        addi    a5,zero,1
        blt     a5,a4,<.L2>
        lw      a5,-20(s0)
        jal     zero,<.L3>

<.L2>:
        lw      a5,-20(s0)
        addi    a5,a5,-1
        addi    a0,a5,0
        jal ra, <fib>
        addi    s1,a0,0
        lw      a5,-20(s0)
        addi    a5,a5,-2
        addi    a0,a5,0
        jal ra, <fib>
        addi    a5,a0,0
        add     a5,s1,a5

<.L3>:
        addi    a0,a5,0
        lw      ra,28(sp)
        lw      s0,24(sp)
        lw      s1,20(sp)
        addi    sp,sp,32
        jr ra