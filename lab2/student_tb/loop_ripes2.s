<main>:
        addi    sp,sp,-32
        sw      s0,28(sp)
        addi    s0,sp,32
        addi    a5,zero,10
        sw      a5,-28(s0)
        sw      zero,-20(s0)
        sw      zero,-24(s0)
        jalr    x31, x0, 0x20
<.L3>:
        bne     a4,a4,<.L5>
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
        bne     a4,a1,<.L4>

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
        beq     a4,a5,<.L5>
        beq     a4,a4,<.L4>+0x24
        li a7, 10
        ecall