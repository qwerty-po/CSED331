<main>:
        addi    sp,sp,-32
        sw      s0,28(sp)
        addi    s0,sp,32
        addi    a5,zero,1
        sw      a5,-20(s0)
        lw      a4,-20(s0)
        addi    a5,zero,1
        bne     a4,a5,<.L2>
        lw      a5,-20(s0)
        addi    a5,a5,2
        sw      a5,-20(s0)

<.L2>:
        lw      a4,-20(s0)
        addi    a5,zero,1
        bge     a5,a4,<.L3>
        lw      a5,-20(s0)
        addi    a5,a5,3
        sw      a5,-20(s0)

<.L3>:
        lw      a4,-20(s0)
        addi    a5,zero,9
        blt     a5,a4,<.L4>
        lw      a5,-20(s0)
        addi    a5,a5,4
        sw      a5,-20(s0)

<.L4>:
        lw      a4,-20(s0)
        addi    a5,zero,9
        beq     a4,a5,<.L6>
        lw      a5,-20(s0)
        slli    a5,a5,0x2
        sw      a5,-20(s0)

<.L6>:
        addi    zero,zero,0
        lw      s0,28(sp)
        addi    sp,sp,32
        # Exit program
        li a7, 10
        ecall