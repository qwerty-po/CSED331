<main>:
        addi    sp,sp,-32
        sw      s0,28(sp)
        addi    s0,sp,32
        addi    a5,zero,10
        sw      a5,-28(s0)
        sw      zero,-20(s0)
        sw      zero,-24(s0)
        jal     zero,<.L2>

<.L3>:
        lw      a4,-20(s0)
        lw      a5,-24(s0)
        add     a5,a4,a5
        sw      a5,-20(s0)
        lw      a5,-24(s0)
        addi    a5,a5,1
        sw      a5,-24(s0)

<.L2>:
        lw      a4,-24(s0)
        lw      a5,-28(s0)
        blt     a4,a5,<.L3>
        sw      zero,-24(s0)
        jal     zero,<.L4>

<.L5>:
        lw      a4,-20(s0)
        lw      a5,-24(s0)
        add     a5,a4,a5
        sw      a5,-20(s0)
        lw      a5,-24(s0)
        addi    a5,a5,1
        sw      a5,-24(s0)

<.L4>:
        lw      a4,-24(s0)
        addi    a5,zero,9
        bge     a5,a4,<.L5>
        lw      a6,-20(s0)
        addi    zero,zero,0
        lw      s0,28(sp)
        addi    sp,sp,32
        li a7, 10
        ecall