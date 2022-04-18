<main>:
        addi    sp,sp,-48
        sw      ra,44(sp)
        sw      s0,40(sp)
        addi    s0,sp,48
        addi    a5,zero,19
        sw      a5,-20(s0)
        addi    a5,zero,14
        sw      a5,-24(s0)
        lw      a1,-24(s0)
        lw      a0,-20(s0)
        jal		ra, <add_func>
        sw      a0,-32(s0)
        lw      a4,-32(s0)
        lw      a5,-20(s0)
        add     a5,a4,a5
        sw      a5,-36(s0)
        sw      zero,-28(s0)
        jal     zero,<.L4>

<add_func>:
        addi    sp,sp,-32
        sw      s0,28(sp)
        addi    s0,sp,32
        sw      a0,-20(s0)
        sw      a1,-24(s0)
        lw      a4,-20(s0)
        lw      a5,-24(s0)
        add     a5,a4,a5
        addi    a0,a5,0
        lw      s0,28(sp)
        addi    sp,sp,32
        jr		ra

<.L9>:
        lw      a4,-20(s0)
        lw      a5,-24(s0)
        bge     a5,a4,<.L5>
        lw      a5,-20(s0)
        addi    a5,a5,-1
        sw      a5,-20(s0)
        jal     zero,<.L6>

<.L5>:
        lw      a4,-20(s0)
        lw      a5,-24(s0)
        beq     a4,a5,<.L10>
        lw      a5,-24(s0)
        addi    a5,a5,-1
        sw      a5,-24(s0)

<.L6>:
        lw      a5,-28(s0)
        addi    a5,a5,1
        sw      a5,-28(s0)

<.L4>:
        lw      a4,-28(s0)
        addi    a5,zero,255
        bge     a5,a4,<.L9>
        jal     zero,<.L11>

<.L10>:
        addi    zero,zero,0

<.L11>:
        addi    zero,zero,0
        lw      ra,44(sp)
        lw      s0,40(sp)
        addi    sp,sp,48
        li		a7, 10
		ecall